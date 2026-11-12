//! High-level robot controller with smooth motor ramping.
//!
//! The [`Robot`] struct wraps a [`MotorDriver`] and provides:
//! - Speed commands as normalized `f32` values in the range `[-1.0, 1.0]`
//! - Smooth acceleration/deceleration via a background task
//! - Configurable motor factors (e.g., to correct for motor polarity)
//! - Graceful shutdown via a cancellation token

use std::sync::Arc;
use std::time::Duration;

use tokio::sync::Mutex;
use tokio::task::JoinHandle;
use tokio::time::Instant;
use tokio_util::sync::CancellationToken;
use tracing::{debug, info, warn};

use crate::error::{MotorError, RobotError};
use crate::motor::{Motor, MotorDriver};

/// Configuration for a [`Robot`] instance.
///
/// Start with [`Default`] and override individual fields. [`Robot::new`] validates
/// the configuration before starting its background task.
#[derive(Debug, Clone)]
pub struct RobotConfig {
    /// Finite left motor factor (`-1.0` reverses polarity, zero disables demand).
    /// The factored demand is clamped to `[-1.0, 1.0]`.
    pub left_motor_factor: f32,
    /// Finite right motor factor (`-1.0` reverses polarity, zero disables demand).
    /// The factored demand is clamped to `[-1.0, 1.0]`.
    pub right_motor_factor: f32,
    /// Which motor is the left motor.
    pub left_motor: Motor,
    /// Which motor is the right motor (must differ from the left motor).
    pub right_motor: Motor,
    /// Positive, finite nominal power change per interval. The slew rate is
    /// step / interval in seconds; elapsed-time updates never exceed this step.
    pub adjustment_step: f32,
    /// Positive interval between motor adjustment ticks. Step divided by interval
    /// in seconds must also be positive and finite in `f32`.
    pub adjustment_interval: Duration,
}

impl RobotConfig {
    fn validate(&self) -> Result<(), RobotError> {
        let invalid = |reason: &str| {
            RobotError::Motor(MotorError::Init(format!("invalid RobotConfig: {reason}")))
        };
        if self.left_motor == self.right_motor {
            return Err(invalid("left_motor and right_motor must differ"));
        }
        if self.adjustment_interval.is_zero() {
            return Err(invalid("adjustment_interval must be positive"));
        }
        if !self.adjustment_step.is_finite() || self.adjustment_step <= 0.0 {
            return Err(invalid("adjustment_step must be finite and positive"));
        }
        let rate = self.adjustment_step / self.adjustment_interval.as_secs_f32();
        if !rate.is_finite() || rate <= 0.0 {
            return Err(invalid(
                "adjustment_step / adjustment_interval must yield a finite, positive f32 rate",
            ));
        }
        if !self.left_motor_factor.is_finite() {
            return Err(invalid("left_motor_factor must be finite"));
        }
        if !self.right_motor_factor.is_finite() {
            return Err(invalid("right_motor_factor must be finite"));
        }
        Ok(())
    }
}

impl Default for RobotConfig {
    fn default() -> Self {
        Self {
            left_motor_factor: 1.0,
            right_motor_factor: 1.0,
            left_motor: Motor::Motor1,
            right_motor: Motor::Motor2,
            adjustment_step: 0.1,
            adjustment_interval: Duration::from_millis(10),
        }
    }
}

/// Controller lifecycle, independent of whether electrical stopping succeeded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum RobotPhase {
    /// Accepting commands; this is not a hardware self-test.
    Running,
    /// Admission is closed and adjustment/cleanup is still finishing.
    ShuttingDown,
    /// Normal shutdown completed without a reported stop failure.
    Shutdown,
    /// A latched task/output/cleanup failure; recovery needs a new robot.
    Faulted,
}

/// What the controller can establish about the final output state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum RobotOutputState {
    /// No successful final stop has been established, or stopping failed.
    /// Also used while running, including immediately after command acceptance.
    Unknown,
    /// The final driver `stop_all` returned success. This is not sensor feedback
    /// or a claim of mechanical standstill, and excludes external driver writes.
    Stopped,
}

/// Bounded timing diagnostics; counters saturate instead of keeping a history.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
#[non_exhaustive]
pub struct RobotTiming {
    /// Updates whose actual elapsed time exceeded the configured interval,
    /// including ordinary timer jitter. This does not latch a controller fault.
    pub overruns: u64,
    /// Scheduled ticks skipped rather than replayed after a delayed update/I/O.
    pub skipped_ticks: u64,
    /// Actual elapsed time before the last update, before the ramp cap.
    /// Includes waiting for the driver lock; zero until the first update.
    pub last_elapsed: Duration,
    /// Largest observed update spacing, before the ramp cap.
    pub max_elapsed: Duration,
}

impl RobotTiming {
    fn record_elapsed(&mut self, elapsed: Duration, interval: Duration) {
        self.last_elapsed = elapsed;
        self.max_elapsed = self.max_elapsed.max(elapsed);
        if elapsed > interval {
            self.overruns = self.overruns.saturating_add(1);
        }
    }
}

/// A read-only snapshot obtained from [`Robot::status`].
///
/// Keeps the first background failure and final cleanup failure separately.
/// Each diagnostic is limited to 4096 UTF-8 bytes (with a truncation marker);
/// repeated rejected commands do not append an error history.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct RobotStatus {
    /// Current admission/lifecycle state. Faults remain latched after shutdown.
    pub phase: RobotPhase,
    /// Result of the final stop, not a measurement of motor motion.
    pub output: RobotOutputState,
    /// First background output or task failure, if one occurred.
    pub fault: Option<String>,
    /// Final stop failure, if one occurred.
    pub cleanup_error: Option<String>,
    /// Whether cleanup has finished, even if its stop attempt failed.
    pub cleanup_complete: bool,
    /// Observed update spacing and skipped deadlines, retained after shutdown.
    pub timing: RobotTiming,
}

struct ControlState {
    targets: (f32, f32),
    status: RobotStatus,
}

impl ControlState {
    fn check_admission(&self, cancelled: bool) -> Result<(), RobotError> {
        if let Some(diagnostic) = &self.status.fault {
            let cleanup = self
                .status
                .cleanup_error
                .as_ref()
                .map_or(String::new(), |error| format!("; cleanup: {error}"));
            return Err(MotorError::Pwm(format!("robot faulted: {diagnostic}{cleanup}")).into());
        }
        if let Some(diagnostic) = &self.status.cleanup_error {
            return Err(MotorError::Pwm(format!("robot cleanup failed: {diagnostic}")).into());
        }
        if cancelled {
            return Err(RobotError::Shutdown);
        }
        Ok(())
    }
}

/// A two-motor differential-drive robot controller.
///
/// The robot smoothly ramps motor speeds from the current value to the target
/// in a background task. Call [`drive`](Robot::drive) to set target speeds and
/// [`stop`](Robot::stop) to decelerate to zero.
///
/// # Shutdown
///
/// Await [`shutdown`](Robot::shutdown) to close command admission, join the
/// adjustment task and finish the final stop attempt. Stop failures are logged
/// and retained in [`status`](Robot::status).
/// Dropping the last public handle requests best-effort asynchronous cleanup
/// while the Tokio runtime remains alive; it does not wait for motor I/O.
pub struct Robot {
    state: Arc<RobotState>,
    // The cleanup task owns/joins the adjustment task. Keep its handle here
    // until an await finishes; cancelling one waiter must not detach its join.
    cleanup: Mutex<Option<JoinHandle<()>>>,
}

// Tasks own only this state, never the public Robot or its cleanup join handle.
struct RobotState {
    motor_board: Arc<Mutex<dyn MotorDriver>>,
    // Serializes paired target updates/snapshots and command admission with
    // shutdown cancellation. Never hold this lock during driver I/O.
    control: Mutex<ControlState>,
    config: RobotConfig,
    cancel_token: CancellationToken,
}

impl Robot {
    /// Create a new robot with the given motor driver and configuration.
    ///
    /// Spawns an adjustment task and its cleanup task. They share internal state
    /// without retaining the public handle. Shutdown or last-handle drop requests
    /// cancellation; cleanup joins adjustment before attempting to stop all motors.
    ///
    /// # Errors
    ///
    /// Returns [`RobotError::Motor`] wrapping [`MotorError::Init`] if the
    /// configuration is invalid. No controller task or driver writes are started
    /// on validation failure; constructing the driver itself may access hardware.
    pub fn new(
        motor_board: Arc<Mutex<dyn MotorDriver>>,
        config: RobotConfig,
    ) -> Result<Arc<Self>, RobotError> {
        config.validate()?;
        let cancel_token = CancellationToken::new();
        let state = Arc::new(RobotState {
            motor_board,
            control: Mutex::new(ControlState {
                targets: (0.0, 0.0),
                status: RobotStatus {
                    phase: RobotPhase::Running,
                    output: RobotOutputState::Unknown,
                    fault: None,
                    cleanup_error: None,
                    cleanup_complete: false,
                    timing: RobotTiming::default(),
                },
            }),
            config,
            cancel_token,
        });

        let adjustment = tokio::spawn(state.clone().smooth_motor_adjustment());
        let cleanup_state = state.clone();
        let cleanup = tokio::spawn(async move {
            if let Err(error) = adjustment.await {
                cleanup_state
                    .latch_fault(format!("motor adjustment task failed: {error}"))
                    .await;
            }
            // Also close admission if adjustment terminated unexpectedly.
            cleanup_state.close_admission().await;
            let result = cleanup_state.motor_board.lock().await.stop_all();
            cleanup_state.finish_cleanup(result).await;
        });
        let robot = Arc::new(Self {
            state,
            cleanup: Mutex::new(Some(cleanup)),
        });

        info!("robot initialized");
        Ok(robot)
    }

    /// Set target motor speeds.
    ///
    /// Finite values are clamped to `[-1.0, 1.0]`, multiplied by the motor factors
    /// from [`RobotConfig`], then clamped again to the supported output range.
    /// A non-finite input rejects the entire command without changing targets.
    /// Both targets are accepted atomically; success means target acceptance,
    /// not hardware application. The task ramps each motor independently and
    /// writes the motors sequentially from one paired target snapshot.
    ///
    /// # Errors
    ///
    /// Returns [`RobotError::Shutdown`] if admission closed without a fault.
    /// A latched failure returns [`RobotError::Motor`] wrapping [`MotorError::Pwm`]
    /// with the retained diagnostic; it takes precedence over shutdown.
    /// Returns [`RobotError::Motor`] wrapping [`MotorError::Pwm`] for a non-finite
    /// input, retaining the existing error API.
    pub async fn drive(&self, left: f32, right: f32) -> Result<(), RobotError> {
        let mut control = self.state.control.lock().await;
        control.check_admission(self.state.cancel_token.is_cancelled())?;

        if !left.is_finite() || !right.is_finite() {
            return Err(MotorError::Pwm(
                "invalid drive command: left and right must both be finite".into(),
            )
            .into());
        }

        let left = scaled_demand(left, self.state.config.left_motor_factor);
        let right = scaled_demand(right, self.state.config.right_motor_factor);

        control.targets = (left, right);
        drop(control);

        debug!(left, right, "drive target set");
        Ok(())
    }

    /// Atomically set both target speeds to zero (motors ramp down independently).
    ///
    /// # Errors
    ///
    /// Returns [`RobotError::Shutdown`] after normal admission closure, or
    /// [`RobotError::Motor`] wrapping [`MotorError::Pwm`] with a latched failure.
    pub async fn stop(&self) -> Result<(), RobotError> {
        let mut control = self.state.control.lock().await;
        control.check_admission(self.state.cancel_token.is_cancelled())?;

        control.targets = (0.0, 0.0);
        drop(control);

        debug!("stop requested");
        Ok(())
    }

    /// Read a consistent snapshot of lifecycle, output certainty and failures.
    ///
    /// Does not perform I/O. A successful drive accepts targets only; it does
    /// not confirm hardware application. A fault stays visible after cleanup
    /// and explicit shutdown, even when the final stop succeeds.
    pub async fn status(&self) -> RobotStatus {
        self.state.control.lock().await.status.clone()
    }

    /// Close command admission, join adjustment, then finish the final stop attempt.
    ///
    /// Repeated/concurrent calls wait for the same cleanup. Once admission closes,
    /// dropping a shutdown future does not cancel cleanup. No adjustment writes
    /// occur after this method returns. It does not ramp down before stopping.
    ///
    /// Driver failures are logged and retained in [`status`](Self::status);
    /// the legacy `()` return value does not
    /// confirm electrical stopping or mechanical standstill. An in-flight driver
    /// operation must finish before cleanup can proceed; blocked I/O can delay
    /// shutdown indefinitely. The Tokio runtime must remain alive for cleanup.
    pub async fn shutdown(&self) {
        info!("shutting down robot");
        self.state.close_admission().await;

        let mut cleanup = self.cleanup.lock().await;
        if let Some(task) = cleanup.as_mut() {
            // Await by reference: cancellation releases this mutex but leaves
            // the join handle available to the next caller.
            if let Err(error) = task.await {
                warn!("robot cleanup task failed: {error}");
            }
            *cleanup = None;
        }
    }
}

impl Drop for Robot {
    fn drop(&mut self) {
        // No command can borrow Robot once its last Arc is dropped. The cleanup
        // task is already running, so Drop needs neither async I/O nor a runtime
        // handle. Dropping its JoinHandle detaches it rather than aborting it.
        self.state.cancel_token.cancel();
    }
}

impl RobotState {
    async fn close_admission(&self) {
        // Serialize with both command validation and paired target replacement.
        let mut control = self.control.lock().await;
        if control.status.phase == RobotPhase::Running {
            control.status.phase = RobotPhase::ShuttingDown;
        }
        self.cancel_token.cancel();
    }

    async fn latch_fault(&self, diagnostic: String) {
        warn!("{diagnostic}");
        let mut control = self.control.lock().await;
        if control.status.fault.is_none() {
            control.status.fault = Some(bounded_diagnostic(diagnostic));
        }
        control.targets = (0.0, 0.0);
        control.status.phase = RobotPhase::Faulted;
        control.status.output = RobotOutputState::Unknown;
        self.cancel_token.cancel();
    }

    async fn finish_cleanup(&self, result: Result<(), MotorError>) {
        let mut control = self.control.lock().await;
        match result {
            Ok(()) => {
                control.status.output = RobotOutputState::Stopped;
                if control.status.phase != RobotPhase::Faulted {
                    control.status.phase = RobotPhase::Shutdown;
                }
            }
            Err(error) => {
                warn!("failed to stop motors during shutdown: {error}");
                control.targets = (0.0, 0.0);
                control.status.cleanup_error = Some(bounded_diagnostic(error.to_string()));
                control.status.phase = RobotPhase::Faulted;
                control.status.output = RobotOutputState::Unknown;
            }
        }
        control.status.cleanup_complete = true;
    }

    async fn smooth_motor_adjustment(self: Arc<Self>) {
        let mut current_left_speed: f32 = 0.0;
        let mut current_right_speed: f32 = 0.0;
        let step = self.config.adjustment_step;
        let interval = self.config.adjustment_interval;
        let mut previous_update = Instant::now();
        // Preserve the initial one-interval delay. A valid but unrepresentably
        // distant deadline waits for cancellation instead of overflowing Instant.
        let mut deadline = previous_update.checked_add(interval);

        loop {
            tokio::select! {
                biased;
                () = self.cancel_token.cancelled() => {
                    debug!("motor adjustment task cancelled");
                    break;
                }
                () = wait_for_tick(deadline) => {}
            }

            let motor_board = tokio::select! {
                biased;
                () = self.cancel_token.cancelled() => break,
                board = self.motor_board.lock() => board,
            };

            // Sample elapsed time after lock waits, immediately before computing
            // this output batch. Never hold the state lock during driver I/O.
            let mut control = self.control.lock().await;
            if self.cancel_token.is_cancelled() {
                break;
            }
            let now = Instant::now();
            let elapsed = now.duration_since(previous_update);
            previous_update = now;
            control.status.timing.record_elapsed(elapsed, interval);
            let (target_left, target_right) = control.targets;
            drop(control);

            let change = elapsed_step(step, interval, elapsed);
            current_left_speed = ramp_toward(current_left_speed, target_left, change);
            current_right_speed = ramp_toward(current_right_speed, target_right, change);
            let left_pwm = speed_to_pwm(current_left_speed);
            let right_pwm = speed_to_pwm(current_right_speed);

            if let Err(e) = apply_motor_speed(
                &*motor_board,
                self.config.left_motor,
                left_pwm,
                current_left_speed,
            ) {
                drop(motor_board);
                self.latch_fault(format!("left motor {:?}: {e}", self.config.left_motor))
                    .await;
                break;
            }

            if let Err(e) = apply_motor_speed(
                &*motor_board,
                self.config.right_motor,
                right_pwm,
                current_right_speed,
            ) {
                drop(motor_board);
                self.latch_fault(format!("right motor {:?}: {e}", self.config.right_motor))
                    .await;
                break;
            }
            drop(motor_board);

            let mut control = self.control.lock().await;
            if let Some(previous_deadline) = deadline {
                let (next, skipped) = next_tick(previous_deadline, interval, Instant::now());
                deadline = next;
                control.status.timing.skipped_ticks =
                    control.status.timing.skipped_ticks.saturating_add(skipped);
            }
        }
    }
}

async fn wait_for_tick(deadline: Option<Instant>) {
    match deadline {
        Some(deadline) => tokio::time::sleep_until(deadline).await,
        None => std::future::pending::<()>().await,
    }
}

/// Advance the original deadline grid strictly beyond now, including delays
/// shorter than a timer's resolution. Never emit a burst of overdue ticks.
fn next_tick(deadline: Instant, interval: Duration, now: Instant) -> (Option<Instant>, u64) {
    let elapsed = now.saturating_duration_since(deadline);
    let skipped = elapsed.as_nanos() / interval.as_nanos();
    let remainder = elapsed.as_nanos() % interval.as_nanos();
    #[expect(
        clippy::cast_possible_truncation,
        reason = "remainder is below a Duration; fractional nanoseconds are below 1e9"
    )]
    let remainder = Duration::new(
        (remainder / 1_000_000_000) as u64,
        (remainder % 1_000_000_000) as u32,
    );
    (
        now.max(deadline)
            .checked_add(interval.saturating_sub(remainder)),
        u64::try_from(skipped).unwrap_or(u64::MAX),
    )
}

/// Rate times elapsed time, capped at one interval and at the configured step.
fn elapsed_step(step: f32, interval: Duration, elapsed: Duration) -> f32 {
    let rate = f64::from(step) / interval.as_secs_f64();
    let change = (rate * elapsed.min(interval).as_secs_f64()).min(f64::from(step));
    #[expect(
        clippy::cast_possible_truncation,
        reason = "nonnegative change is bounded by the validated finite f32 step"
    )]
    let change = change as f32;
    change
}

fn bounded_diagnostic(mut diagnostic: String) -> String {
    const LIMIT: usize = 4096;
    const MARKER: &str = " [truncated]";
    if diagnostic.len() > LIMIT {
        let mut end = LIMIT - MARKER.len();
        while !diagnostic.is_char_boundary(end) {
            end -= 1;
        }
        diagnostic.truncate(end);
        diagnostic.push_str(MARKER);
        diagnostic.shrink_to_fit();
    }
    diagnostic
}

/// Inputs and factors have been validated; clamp before and after scaling.
fn scaled_demand(input: f32, factor: f32) -> f32 {
    let demand = f64::from(input.clamp(-1.0, 1.0)) * f64::from(factor);
    #[expect(
        clippy::cast_possible_truncation,
        reason = "bounded demand fits in f32"
    )]
    let demand = demand.clamp(-1.0, 1.0) as f32;
    demand
}

/// Ramp `current` toward `target` by at most `step`.
fn ramp_toward(current: f32, target: f32, step: f32) -> f32 {
    if (current - target).abs() > step {
        if current < target {
            current + step
        } else {
            current - step
        }
    } else {
        target
    }
}

/// Convert a normalized speed `[-1.0, 1.0]` to a PWM duty cycle `[0, 4095]`.
fn speed_to_pwm(speed: f32) -> u16 {
    #[expect(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "speed is clamped to [0.0, 1.0] after abs(), result fits in u16"
    )]
    let pwm = (speed.abs() * 4095.0).round() as u16;
    pwm
}

/// Apply the computed PWM value to a motor, choosing direction or stopping.
fn apply_motor_speed(
    board: &dyn MotorDriver,
    motor: Motor,
    pwm: u16,
    speed: f32,
) -> Result<(), MotorError> {
    if speed > 0.0 {
        board.set_motor_speed(motor, pwm, true)
    } else if speed < 0.0 {
        board.set_motor_speed(motor, pwm, false)
    } else {
        board.stop_motor(motor)
    }
}

#[cfg(test)]
mod lifecycle_tests;

#[cfg(test)]
mod timing_tests;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mock::{MockMotorDriver, MotorCommand};

    // No Tokio runtime: invalid configurations must return before tokio::spawn.
    #[test]
    fn invalid_config_never_starts_controller() {
        let mut cases = vec![
            (
                RobotConfig {
                    right_motor: Motor::Motor1,
                    ..RobotConfig::default()
                },
                "must differ",
            ),
            (
                RobotConfig {
                    adjustment_interval: Duration::ZERO,
                    ..RobotConfig::default()
                },
                "adjustment_interval",
            ),
            (
                RobotConfig {
                    adjustment_step: f32::MAX,
                    ..RobotConfig::default()
                },
                "rate",
            ),
            (
                RobotConfig {
                    adjustment_step: f32::from_bits(1),
                    adjustment_interval: Duration::MAX,
                    ..RobotConfig::default()
                },
                "rate",
            ),
        ];
        for step in [0.0, -0.1, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            cases.push((
                RobotConfig {
                    adjustment_step: step,
                    ..RobotConfig::default()
                },
                "adjustment_step",
            ));
        }
        for factor in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            cases.push((
                RobotConfig {
                    left_motor_factor: factor,
                    ..RobotConfig::default()
                },
                "left_motor_factor",
            ));
            cases.push((
                RobotConfig {
                    right_motor_factor: factor,
                    ..RobotConfig::default()
                },
                "right_motor_factor",
            ));
        }
        for (config, expected) in cases {
            let driver = Arc::new(Mutex::new(MockMotorDriver::new()));
            let Err(RobotError::Motor(MotorError::Init(message))) =
                Robot::new(driver.clone(), config)
            else {
                panic!("expected configuration error for {expected}");
            };
            assert!(message.contains(expected), "{message}");
            assert_eq!(Arc::strong_count(&driver), 1);
            assert_eq!(driver.try_lock().unwrap().command_count(), 0);
        }
    }

    #[tokio::test]
    async fn invalid_drive_preserves_both_targets() {
        let driver = Arc::new(Mutex::new(MockMotorDriver::new()));
        let robot = Robot::new(driver.clone(), RobotConfig::default()).unwrap();
        robot.drive(0.25, -0.5).await.unwrap();
        for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            for (left, right) in [(invalid, 0.75), (0.75, invalid), (invalid, invalid)] {
                assert!(matches!(
                    robot.drive(left, right).await,
                    Err(RobotError::Motor(MotorError::Pwm(_)))
                ));
                assert_eq!(robot.state.control.lock().await.targets, (0.25, -0.5));
            }
        }
        robot.shutdown().await;
    }

    #[tokio::test]
    async fn finite_factors_scale_then_saturate() {
        for (factor, input, expected) in [
            (2.0, 0.75, 1.0),
            (-2.0, 0.75, -1.0),
            (0.5, 2.0, 0.5), // Clamp input before applying the factor.
            (0.0, f32::MAX, 0.0),
            (f32::MAX, f32::MAX, 1.0),
            (f32::MAX, -f32::MAX, -1.0),
            (-f32::MAX, -f32::MAX, 1.0),
            (f32::MAX, 0.0, 0.0),
        ] {
            let driver = Arc::new(Mutex::new(MockMotorDriver::new()));
            let config = RobotConfig {
                left_motor_factor: factor,
                right_motor_factor: factor,
                ..RobotConfig::default()
            };
            let robot = Robot::new(driver, config).unwrap();
            robot.drive(input, input).await.unwrap();
            assert_eq!(
                robot.state.control.lock().await.targets,
                (expected, expected)
            );
            robot.shutdown().await;
        }
    }

    #[test]
    fn ramp_toward_moves_by_step() {
        assert!((ramp_toward(0.0, 1.0, 0.1) - 0.1).abs() < f32::EPSILON);
        assert!((ramp_toward(1.0, 0.0, 0.1) - 0.9).abs() < f32::EPSILON);
    }

    #[test]
    fn ramp_toward_snaps_when_close() {
        assert!((ramp_toward(0.95, 1.0, 0.1) - 1.0).abs() < f32::EPSILON);
    }

    #[test]
    fn speed_to_pwm_converts_correctly() {
        assert_eq!(speed_to_pwm(0.0), 0);
        assert_eq!(speed_to_pwm(1.0), 4095);
        assert_eq!(speed_to_pwm(-1.0), 4095);
        assert_eq!(speed_to_pwm(0.5), 2048);
    }

    #[test]
    fn apply_motor_speed_forward() {
        let driver = MockMotorDriver::new();
        apply_motor_speed(&driver, Motor::Motor1, 2048, 0.5).unwrap();
        assert_eq!(
            driver.commands(),
            vec![MotorCommand::SetSpeed {
                motor: Motor::Motor1,
                speed: 2048,
                forward: true,
            }]
        );
    }

    #[test]
    fn apply_motor_speed_reverse() {
        let driver = MockMotorDriver::new();
        apply_motor_speed(&driver, Motor::Motor1, 2048, -0.5).unwrap();
        assert_eq!(
            driver.commands(),
            vec![MotorCommand::SetSpeed {
                motor: Motor::Motor1,
                speed: 2048,
                forward: false,
            }]
        );
    }

    #[test]
    fn apply_motor_speed_stop() {
        let driver = MockMotorDriver::new();
        apply_motor_speed(&driver, Motor::Motor1, 0, 0.0).unwrap();
        assert_eq!(
            driver.commands(),
            vec![MotorCommand::Stop {
                motor: Motor::Motor1
            }]
        );
    }

    #[tokio::test]
    async fn drive_clamps_input() {
        let driver = Arc::new(Mutex::new(MockMotorDriver::new()));
        let robot = Robot::new(driver.clone(), RobotConfig::default()).unwrap();

        robot.drive(2.0, -3.0).await.unwrap();

        let (left, right) = robot.state.control.lock().await.targets;

        // With default factors (1.0, 1.0), clamped to [-1.0, 1.0]
        assert!((left - 1.0).abs() < f32::EPSILON);
        assert!((right - (-1.0)).abs() < f32::EPSILON);

        robot.shutdown().await;
    }

    #[tokio::test]
    async fn stop_sets_zero_targets() {
        let driver = Arc::new(Mutex::new(MockMotorDriver::new()));
        let robot = Robot::new(driver.clone(), RobotConfig::default()).unwrap();

        robot.drive(0.5, 0.5).await.unwrap();
        robot.stop().await.unwrap();

        let (left, right) = robot.state.control.lock().await.targets;

        assert!((left).abs() < f32::EPSILON);
        assert!((right).abs() < f32::EPSILON);

        robot.shutdown().await;
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn concurrent_drive_stop_snapshots_are_complete_pairs() {
        tokio::time::timeout(Duration::from_secs(5), async {
            let driver = Arc::new(Mutex::new(MockMotorDriver::new()));
            let robot = Robot::new(driver, RobotConfig::default()).unwrap();
            let barrier = Arc::new(tokio::sync::Barrier::new(4));
            let mut writers = tokio::task::JoinSet::new();
            let commands = [Some((0.75, -0.25)), Some((-0.5, 1.0)), None];
            let accepted_pairs = [(0.75, -0.25), (-0.5, 1.0), (0.0, 0.0)];

            for command in commands {
                let robot = robot.clone();
                let barrier = barrier.clone();
                writers.spawn(async move {
                    for _ in 0..128 {
                        barrier.wait().await;
                        match command {
                            Some((left, right)) => robot.drive(left, right).await.unwrap(),
                            None => robot.stop().await.unwrap(),
                        }
                        let snapshot = robot.state.control.lock().await.targets;
                        assert!(
                            accepted_pairs.contains(&snapshot),
                            "mixed pair: {snapshot:?}"
                        );
                        barrier.wait().await;
                    }
                });
            }

            // Observe while both drive calls and stop contend, then again once
            // all three have completed. No ordering between writers is assumed.
            for _ in 0..128 {
                barrier.wait().await;
                let snapshot = robot.state.control.lock().await.targets;
                assert!(
                    accepted_pairs.contains(&snapshot),
                    "mixed pair: {snapshot:?}"
                );
                barrier.wait().await;
                assert!(accepted_pairs.contains(&robot.state.control.lock().await.targets));
            }
            while let Some(result) = writers.join_next().await {
                result.unwrap();
            }
            robot.shutdown().await;
        })
        .await
        .expect("concurrent commands deadlocked");
    }

    // Poll once to deterministically enqueue a waiter on the held state lock.
    async fn assert_pending<F: std::future::Future>(mut future: std::pin::Pin<&mut F>) {
        std::future::poll_fn(|cx| {
            assert!(future.as_mut().poll(cx).is_pending());
            std::task::Poll::Ready(())
        })
        .await;
    }

    #[tokio::test]
    async fn waiting_commands_recheck_admission_after_shutdown() {
        tokio::time::timeout(Duration::from_secs(5), async {
            let driver = Arc::new(Mutex::new(MockMotorDriver::new()));
            let robot = Robot::new(driver.clone(), RobotConfig::default()).unwrap();
            robot.drive(0.25, -0.5).await.unwrap();
            let targets = robot.state.control.lock().await;
            // Hold I/O too: admission must close before shutdown can stop the
            // board, without keeping the target lock held during that I/O wait.
            let board = driver.lock().await;
            let shutdown = robot.shutdown();
            let drive = robot.drive(-0.75, 1.0);
            let stop = robot.stop();
            tokio::pin!(shutdown, drive, stop);

            // Tokio's mutex admits waiters in FIFO order. All three calls start
            // before cancellation, but shutdown will acquire the state first.
            assert_pending(shutdown.as_mut()).await;
            assert_pending(drive.as_mut()).await;
            assert_pending(stop.as_mut()).await;
            assert!(!robot.state.cancel_token.is_cancelled());
            drop(targets);

            assert_pending(shutdown.as_mut()).await;
            assert!(robot.state.cancel_token.is_cancelled());
            assert!(matches!(drive.await, Err(RobotError::Shutdown)));
            assert!(matches!(stop.await, Err(RobotError::Shutdown)));
            assert_eq!(robot.state.control.lock().await.targets, (0.25, -0.5));
            drop(board);
            shutdown.await;
        })
        .await
        .expect("shutdown admission deadlocked");
    }

    #[tokio::test(start_paused = true)]
    async fn adjustment_applies_complete_target_pairs() {
        let driver = Arc::new(Mutex::new(MockMotorDriver::new()));
        let robot = Robot::new(
            driver.clone(),
            RobotConfig {
                // Reach any target in one tick, so writes reveal the snapshot
                // directly without intermediate per-motor ramp values.
                adjustment_step: 2.0,
                ..RobotConfig::default()
            },
        )
        .unwrap();
        for command in [Some((0.75, -0.25)), None, Some((-0.5, 1.0))] {
            let (left, right) = if let Some((left, right)) = command {
                robot.drive(left, right).await.unwrap();
                (left, right)
            } else {
                robot.stop().await.unwrap();
                (0.0, 0.0)
            };
            driver.lock().await.clear();
            tokio::task::yield_now().await;
            tokio::time::advance(Duration::from_millis(10)).await;
            tokio::task::yield_now().await;

            let expected = MockMotorDriver::new();
            apply_motor_speed(&expected, Motor::Motor1, speed_to_pwm(left), left).unwrap();
            apply_motor_speed(&expected, Motor::Motor2, speed_to_pwm(right), right).unwrap();
            assert_eq!(driver.lock().await.commands(), expected.commands());
        }
        robot.shutdown().await;
    }

    #[tokio::test]
    async fn shutdown_stops_motors() {
        let driver = Arc::new(Mutex::new(MockMotorDriver::new()));
        let robot = Robot::new(driver.clone(), RobotConfig::default()).unwrap();

        robot.shutdown().await;

        // After shutdown, drive should return error
        assert!(robot.drive(0.5, 0.5).await.is_err());
    }

    #[tokio::test]
    async fn motor_factors_applied() {
        let driver = Arc::new(Mutex::new(MockMotorDriver::new()));
        let config = RobotConfig {
            left_motor_factor: -1.0,
            right_motor_factor: 1.0,
            ..RobotConfig::default()
        };
        let robot = Robot::new(driver.clone(), config).unwrap();

        robot.drive(0.5, 0.5).await.unwrap();

        let (left, right) = robot.state.control.lock().await.targets;

        // Left factor is -1.0, so 0.5 * -1.0 = -0.5
        assert!((left - (-0.5)).abs() < f32::EPSILON);
        assert!((right - 0.5).abs() < f32::EPSILON);

        robot.shutdown().await;
    }
}
