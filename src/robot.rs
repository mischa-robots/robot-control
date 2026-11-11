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
    /// Positive, finite speed adjustment step per tick (controls ramp smoothness).
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

/// A two-motor differential-drive robot controller.
///
/// The robot smoothly ramps motor speeds from the current value to the target
/// in a background task. Call [`drive`](Robot::drive) to set target speeds and
/// [`stop`](Robot::stop) to decelerate to zero.
///
/// # Shutdown
///
/// Call [`shutdown`](Robot::shutdown) to cancel the background task and stop all motors.
/// The robot also stops motors when dropped, but explicit shutdown is preferred.
pub struct Robot {
    motor_board: Arc<Mutex<dyn MotorDriver>>,
    // Serializes paired target updates/snapshots and command admission with
    // shutdown cancellation. Never hold this lock during driver I/O.
    target_speeds: Mutex<(f32, f32)>,
    config: RobotConfig,
    cancel_token: CancellationToken,
}

impl Robot {
    /// Create a new robot with the given motor driver and configuration.
    ///
    /// Spawns a background task for smooth motor ramping. The task runs until
    /// [`shutdown`](Robot::shutdown) is called or the [`Robot`] is dropped.
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
        let robot = Arc::new(Self {
            motor_board,
            target_speeds: Mutex::new((0.0, 0.0)),
            config,
            cancel_token,
        });

        let robot_clone = Arc::clone(&robot);
        tokio::spawn(async move {
            robot_clone.smooth_motor_adjustment().await;
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
    /// Returns [`RobotError::Shutdown`] if the robot has been shut down.
    /// Returns [`RobotError::Motor`] wrapping [`MotorError::Pwm`] for a non-finite
    /// input, retaining the existing error API.
    pub async fn drive(&self, left: f32, right: f32) -> Result<(), RobotError> {
        let mut targets = self.target_speeds.lock().await;
        if self.cancel_token.is_cancelled() {
            return Err(RobotError::Shutdown);
        }

        if !left.is_finite() || !right.is_finite() {
            return Err(MotorError::Pwm(
                "invalid drive command: left and right must both be finite".into(),
            )
            .into());
        }

        let left = scaled_demand(left, self.config.left_motor_factor);
        let right = scaled_demand(right, self.config.right_motor_factor);

        *targets = (left, right);
        drop(targets);

        debug!(left, right, "drive target set");
        Ok(())
    }

    /// Atomically set both target speeds to zero (motors ramp down independently).
    ///
    /// # Errors
    ///
    /// Returns [`RobotError::Shutdown`] if the robot has been shut down.
    pub async fn stop(&self) -> Result<(), RobotError> {
        let mut targets = self.target_speeds.lock().await;
        if self.cancel_token.is_cancelled() {
            return Err(RobotError::Shutdown);
        }

        *targets = (0.0, 0.0);
        drop(targets);

        debug!("stop requested");
        Ok(())
    }

    /// Cancel the background task and stop all motors immediately.
    pub async fn shutdown(&self) {
        info!("shutting down robot");
        {
            // Waiting commands must check cancellation after acquiring the
            // same lock, so none can replace targets after admission closes.
            let _targets = self.target_speeds.lock().await;
            self.cancel_token.cancel();
        }

        if let Err(e) = self.motor_board.lock().await.stop_all() {
            warn!("failed to stop motors during shutdown: {e}");
        }
    }

    async fn smooth_motor_adjustment(self: Arc<Self>) {
        let mut current_left_speed: f32 = 0.0;
        let mut current_right_speed: f32 = 0.0;
        let step = self.config.adjustment_step;
        let interval = self.config.adjustment_interval;

        loop {
            tokio::select! {
                () = self.cancel_token.cancelled() => {
                    debug!("motor adjustment task cancelled");
                    break;
                }
                () = tokio::time::sleep(interval) => {}
            }

            let (target_left, target_right) = *self.target_speeds.lock().await;

            current_left_speed = ramp_toward(current_left_speed, target_left, step);
            current_right_speed = ramp_toward(current_right_speed, target_right, step);

            let left_pwm = speed_to_pwm(current_left_speed);
            let right_pwm = speed_to_pwm(current_right_speed);

            let motor_board = self.motor_board.lock().await;

            if let Err(e) = apply_motor_speed(
                &*motor_board,
                self.config.left_motor,
                left_pwm,
                current_left_speed,
            ) {
                warn!("left motor error: {e}");
            }

            if let Err(e) = apply_motor_speed(
                &*motor_board,
                self.config.right_motor,
                right_pwm,
                current_right_speed,
            ) {
                warn!("right motor error: {e}");
            }
        }
    }
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
                assert_eq!(*robot.target_speeds.lock().await, (0.25, -0.5));
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
            assert_eq!(*robot.target_speeds.lock().await, (expected, expected));
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

        let (left, right) = *robot.target_speeds.lock().await;

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

        let (left, right) = *robot.target_speeds.lock().await;

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
                        let snapshot = *robot.target_speeds.lock().await;
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
                let snapshot = *robot.target_speeds.lock().await;
                assert!(
                    accepted_pairs.contains(&snapshot),
                    "mixed pair: {snapshot:?}"
                );
                barrier.wait().await;
                assert!(accepted_pairs.contains(&*robot.target_speeds.lock().await));
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
            let targets = robot.target_speeds.lock().await;
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
            assert!(!robot.cancel_token.is_cancelled());
            drop(targets);

            assert_pending(shutdown.as_mut()).await;
            assert!(robot.cancel_token.is_cancelled());
            assert!(matches!(drive.await, Err(RobotError::Shutdown)));
            assert!(matches!(stop.await, Err(RobotError::Shutdown)));
            assert_eq!(*robot.target_speeds.lock().await, (0.25, -0.5));
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

        let (left, right) = *robot.target_speeds.lock().await;

        // Left factor is -1.0, so 0.5 * -1.0 = -0.5
        assert!((left - (-0.5)).abs() < f32::EPSILON);
        assert!((right - 0.5).abs() < f32::EPSILON);

        robot.shutdown().await;
    }
}
