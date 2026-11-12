//! # Robot Control
//!
//! A Rust library for controlling DC motor robots via PWM motor driver boards.
//!
//! This crate provides a hardware-abstracted [`MotorDriver`] trait, allowing you to
//! control robots in production (with real I2C hardware) and in tests (with a mock driver).
//!
//! ## Features
//!
//! - **`pca9685`** — Enables the `Pca9685MotorBoard`
//!   implementation for PCA9685 PWM boards over I2C. Only available on Linux with I2C support.
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use robot_control::{Motor, MotorDriver, Robot, RobotConfig};
//! use std::sync::Arc;
//! use tokio::sync::Mutex;
//!
//! # async fn example() -> Result<(), robot_control::RobotError> {
//! // Use any MotorDriver implementation (mock for testing, Pca9685 for real hardware)
//! let driver: Arc<Mutex<dyn MotorDriver>> = todo!("create your driver");
//! let robot = Robot::new(driver, RobotConfig::default())?;
//! robot.drive(0.5, 0.5).await?;
//! robot.stop().await?;
//! robot.shutdown().await;
//! # Ok(())
//! # }
//! ```

#![forbid(unsafe_code)]

pub mod error;
pub mod motor;
#[cfg(feature = "pca9685")]
pub mod pca9685;
pub mod robot;

pub use error::{MotorError, RobotError};
pub use motor::{Motor, MotorDriver};
#[cfg(feature = "pca9685")]
pub use pca9685::Pca9685MotorBoard;
pub use robot::{Robot, RobotConfig, RobotOutputState, RobotPhase, RobotStatus, RobotTiming};

#[cfg(test)]
pub mod mock;
