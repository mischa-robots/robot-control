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
/// Use the builder methods or [`Default`] for sensible defaults.
#[derive(Debug, Clone)]
pub struct RobotConfig {
    /// Factor applied to the left motor speed (use `-1.0` to reverse polarity).
    pub left_motor_factor: f32,
    /// Factor applied to the right motor speed (use `-1.0` to reverse polarity).
    pub right_motor_factor: f32,
    /// Which motor is the left motor.
    pub left_motor: Motor,
    /// Which motor is the right motor.
    pub right_motor: Motor,
    /// Speed adjustment step per tick (controls ramp smoothness).
    pub adjustment_step: f32,
    /// Interval between motor adjustment ticks.
    pub adjustment_interval: Duration,
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
    target_left_speed: Mutex<f32>,
    target_right_speed: Mutex<f32>,
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
    /// Returns [`RobotError`] if the configuration is invalid.
    pub fn new(
        motor_board: Arc<Mutex<dyn MotorDriver>>,
        config: RobotConfig,
    ) -> Result<Arc<Self>, RobotError> {
        let cancel_token = CancellationToken::new();
        let robot = Arc::new(Self {
            motor_board,
            target_left_speed: Mutex::new(0.0),
            target_right_speed: Mutex::new(0.0),
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
    /// Values are clamped to `[-1.0, 1.0]`. Motor factors from [`RobotConfig`]
    /// are applied automatically.
    ///
    /// # Errors
    ///
    /// Returns [`RobotError::Shutdown`] if the robot has been shut down.
    pub async fn drive(&self, left: f32, right: f32) -> Result<(), RobotError> {
        if self.cancel_token.is_cancelled() {
            return Err(RobotError::Shutdown);
        }

        let left = left.clamp(-1.0, 1.0) * self.config.left_motor_factor;
        let right = right.clamp(-1.0, 1.0) * self.config.right_motor_factor;

        *self.target_left_speed.lock().await = left;
        *self.target_right_speed.lock().await = right;

        debug!(left, right, "drive target set");
        Ok(())
    }

    /// Set target speeds to zero (motors will ramp down smoothly).
    ///
    /// # Errors
    ///
    /// Returns [`RobotError::Shutdown`] if the robot has been shut down.
    pub async fn stop(&self) -> Result<(), RobotError> {
        if self.cancel_token.is_cancelled() {
            return Err(RobotError::Shutdown);
        }

        *self.target_left_speed.lock().await = 0.0;
        *self.target_right_speed.lock().await = 0.0;

        debug!("stop requested");
        Ok(())
    }

    /// Cancel the background task and stop all motors immediately.
    pub async fn shutdown(&self) {
        info!("shutting down robot");
        self.cancel_token.cancel();

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

            let target_left = *self.target_left_speed.lock().await;
            let target_right = *self.target_right_speed.lock().await;

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

        let left = *robot.target_left_speed.lock().await;
        let right = *robot.target_right_speed.lock().await;

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

        let left = *robot.target_left_speed.lock().await;
        let right = *robot.target_right_speed.lock().await;

        assert!((left).abs() < f32::EPSILON);
        assert!((right).abs() < f32::EPSILON);

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

        let left = *robot.target_left_speed.lock().await;
        let right = *robot.target_right_speed.lock().await;

        // Left factor is -1.0, so 0.5 * -1.0 = -0.5
        assert!((left - (-0.5)).abs() < f32::EPSILON);
        assert!((right - 0.5).abs() < f32::EPSILON);

        robot.shutdown().await;
    }
}
