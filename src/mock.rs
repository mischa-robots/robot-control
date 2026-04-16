//! Mock motor driver for testing.
//!
//! Provides [`MockMotorDriver`], which records all motor commands for assertion in tests.

use std::sync::Mutex;

use crate::error::MotorError;
use crate::motor::{Motor, MotorDriver};

/// A recorded motor command.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MotorCommand {
    /// A speed was set on a motor.
    SetSpeed {
        motor: Motor,
        speed: u16,
        forward: bool,
    },
    /// A motor was stopped.
    Stop { motor: Motor },
}

/// A mock motor driver that records all commands for test assertions.
///
/// # Example
///
/// ```
/// use robot_control::mock::MockMotorDriver;
/// use robot_control::{Motor, MotorDriver};
///
/// let driver = MockMotorDriver::new();
/// driver.set_motor_speed(Motor::Motor1, 2048, true).unwrap();
/// assert_eq!(driver.command_count(), 1);
/// ```
pub struct MockMotorDriver {
    commands: Mutex<Vec<MotorCommand>>,
}

impl MockMotorDriver {
    /// Create a new mock driver with an empty command log.
    #[must_use]
    pub fn new() -> Self {
        Self {
            commands: Mutex::new(Vec::new()),
        }
    }

    /// Return a clone of all recorded commands.
    ///
    /// # Panics
    ///
    /// Panics if the internal mutex is poisoned.
    #[must_use]
    pub fn commands(&self) -> Vec<MotorCommand> {
        self.commands.lock().expect("mock mutex poisoned").clone()
    }

    /// Return the number of recorded commands.
    ///
    /// # Panics
    ///
    /// Panics if the internal mutex is poisoned.
    #[must_use]
    pub fn command_count(&self) -> usize {
        self.commands.lock().expect("mock mutex poisoned").len()
    }

    /// Clear all recorded commands.
    ///
    /// # Panics
    ///
    /// Panics if the internal mutex is poisoned.
    pub fn clear(&self) {
        self.commands.lock().expect("mock mutex poisoned").clear();
    }
}

impl Default for MockMotorDriver {
    fn default() -> Self {
        Self::new()
    }
}

impl MotorDriver for MockMotorDriver {
    fn set_motor_speed(&self, motor: Motor, speed: u16, forward: bool) -> Result<(), MotorError> {
        self.commands
            .lock()
            .expect("mock mutex poisoned")
            .push(MotorCommand::SetSpeed {
                motor,
                speed,
                forward,
            });
        Ok(())
    }

    fn stop_motor(&self, motor: Motor) -> Result<(), MotorError> {
        self.commands
            .lock()
            .expect("mock mutex poisoned")
            .push(MotorCommand::Stop { motor });
        Ok(())
    }
}
