//! Error types for the robot-control crate.

use thiserror::Error;

/// Errors that can occur when interacting with a motor driver.
#[derive(Debug, Error)]
pub enum MotorError {
    /// An I2C bus operation failed.
    #[error("I2C error: {0}")]
    I2c(String),

    /// A PWM controller operation failed.
    #[error("PWM error: {0}")]
    Pwm(String),

    /// The motor driver failed to initialize.
    #[error("initialization failed: {0}")]
    Init(String),
}

/// Errors that can occur when controlling the robot.
#[derive(Debug, Error)]
pub enum RobotError {
    /// A motor driver operation failed.
    #[error(transparent)]
    Motor(#[from] MotorError),

    /// The robot has already been shut down.
    #[error("robot has been shut down")]
    Shutdown,
}
