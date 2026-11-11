//! Motor driver abstraction and motor definitions.
//!
//! This module defines the [`MotorDriver`] trait and the [`Motor`] enum.
//! Concrete implementations live in separate modules (e.g., `pca9685` with the feature flag).

use crate::error::MotorError;

/// Identifies a DC motor connected to the motor driver board.
///
/// Most dual-motor robot kits use [`Motor1`](Motor::Motor1) (left) and
/// [`Motor2`](Motor::Motor2) (right). Four-motor configurations use all four.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Motor {
    Motor1,
    Motor2,
    Motor3,
    Motor4,
}

/// Abstraction over a motor driver board.
///
/// Implement this trait for your specific hardware (e.g., PCA9685) or use
/// `MockMotorDriver` (available in test builds) for testing.
pub trait MotorDriver: Send + Sync {
    /// Set the speed and direction of a single motor.
    ///
    /// # Arguments
    ///
    /// * `motor` — Which motor to control.
    /// * `speed` — PWM duty cycle (0–4095).
    /// * `forward` — `true` for forward, `false` for reverse.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError`] if the hardware communication fails.
    fn set_motor_speed(&self, motor: Motor, speed: u16, forward: bool) -> Result<(), MotorError>;

    /// Stop a single motor (set all channels to zero).
    ///
    /// # Errors
    ///
    /// Returns [`MotorError`] if the hardware communication fails.
    fn stop_motor(&self, motor: Motor) -> Result<(), MotorError>;

    /// Stop all motors. The default implementation attempts M1–M4 in order,
    /// even if an earlier motor fails to stop.
    ///
    /// Custom overrides must provide their own exhaustive stopping behavior.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError::Pwm`] if any stop fails, collecting every failed
    /// motor and its original diagnostic. An error means outputs are uncertain.
    fn stop_all(&self) -> Result<(), MotorError> {
        let mut failures = Vec::new();
        for motor in [Motor::Motor1, Motor::Motor2, Motor::Motor3, Motor::Motor4] {
            if let Err(error) = self.stop_motor(motor) {
                failures.push(format!("{motor:?} stop_motor: {error}"));
            }
        }
        if failures.is_empty() {
            Ok(())
        } else {
            Err(MotorError::Pwm(format!(
                "stop_all failed: {}",
                failures.join("; ")
            )))
        }
    }
}
