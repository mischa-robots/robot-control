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

    /// Stop all motors. The default implementation stops each motor individually.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError`] if any hardware communication fails.
    fn stop_all(&self) -> Result<(), MotorError> {
        self.stop_motor(Motor::Motor1)?;
        self.stop_motor(Motor::Motor2)?;
        self.stop_motor(Motor::Motor3)?;
        self.stop_motor(Motor::Motor4)?;
        Ok(())
    }
}
