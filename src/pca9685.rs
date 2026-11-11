//! PCA9685 PWM motor driver board implementation.
//!
//! This module provides [`Pca9685MotorBoard`], a concrete [`MotorDriver`](crate::MotorDriver)
//! implementation for PCA9685-based motor driver boards connected via I2C.
//!
//! **Requires the `pca9685` feature flag.**
//!
//! # Hardware Setup
//!
//! The PCA9685 board controls DC motors through three PWM channels per motor:
//! - A reference channel (always high when active)
//! - A forward channel (PWM duty cycle sets forward speed)
//! - A backward channel (PWM duty cycle sets reverse speed)
//!
//! # Example
//!
//! ```rust,no_run
//! use robot_control::pca9685::Pca9685MotorBoard;
//! use robot_control::{Motor, MotorDriver};
//!
//! let board = Pca9685MotorBoard::new("/dev/i2c-1", 0x60)?;
//! board.set_motor_speed(Motor::Motor1, 2048, true)?;
//! # Ok::<(), robot_control::MotorError>(())
//! ```

use linux_embedded_hal::I2cdev;
use pwm_pca9685::{Channel, Pca9685};

use crate::error::MotorError;
use crate::motor::{Motor, MotorDriver};

/// PWM channel mapping for a single DC motor.
struct DcChannels {
    ref_channel: Channel,
    forward_channel: Channel,
    backward_channel: Channel,
}

/// Returns the PWM channel mapping for a given motor.
const fn channels_for(motor: Motor) -> DcChannels {
    match motor {
        Motor::Motor1 => DcChannels {
            ref_channel: Channel::C8,
            forward_channel: Channel::C9,
            backward_channel: Channel::C10,
        },
        Motor::Motor2 => DcChannels {
            ref_channel: Channel::C13,
            forward_channel: Channel::C11,
            backward_channel: Channel::C12,
        },
        Motor::Motor3 => DcChannels {
            ref_channel: Channel::C2,
            forward_channel: Channel::C3,
            backward_channel: Channel::C4,
        },
        Motor::Motor4 => DcChannels {
            ref_channel: Channel::C7,
            forward_channel: Channel::C5,
            backward_channel: Channel::C6,
        },
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StopWrite {
    On,
    Off,
}

impl StopWrite {
    const fn name(self) -> &'static str {
        match self {
            Self::On => "set_channel_on",
            Self::Off => "set_channel_off",
        }
    }
}

// Keep the stop sequence private and inject only the individual write for tests.
// This is the existing six-write sequence, including its order and zero values.
fn stop_motor_outputs<E: std::fmt::Debug>(
    motor: Motor,
    mut write: impl FnMut(Channel, StopWrite, u16) -> Result<(), E>,
) -> Result<(), MotorError> {
    let channels = channels_for(motor);
    let mut failures = Vec::new();
    for channel in [
        channels.ref_channel,
        channels.forward_channel,
        channels.backward_channel,
    ] {
        for operation in [StopWrite::On, StopWrite::Off] {
            if let Err(error) = write(channel, operation, 0) {
                failures.push(format!(
                    "{motor:?} channel {channel:?} {}(0): {error:?}",
                    operation.name()
                ));
            }
        }
    }
    if failures.is_empty() {
        Ok(())
    } else {
        Err(MotorError::Pwm(format!(
            "stop_motor failed: {}",
            failures.join("; ")
        )))
    }
}

/// A motor driver backed by a PCA9685 PWM controller over I2C.
///
/// This implementation uses [`linux_embedded_hal::I2cdev`] for I2C communication
/// and the [`pwm_pca9685`] crate for PWM control.
///
/// The internal PWM controller is protected by a [`std::sync::Mutex`] for
/// thread-safe access. When used with [`Robot`](crate::Robot), the outer
/// `tokio::sync::Mutex` serializes access, so contention is minimal.
pub struct Pca9685MotorBoard {
    pwm: std::sync::Mutex<Pca9685<I2cdev>>,
}

impl Pca9685MotorBoard {
    /// Create a new `Pca9685MotorBoard` connected to the given I2C bus and address.
    ///
    /// # Arguments
    ///
    /// * `i2c_bus` — Path to the I2C device (e.g., `"/dev/i2c-1"`).
    /// * `address` — I2C address of the PCA9685 board (typically `0x60`).
    ///
    /// # Errors
    ///
    /// Returns [`MotorError::Init`] if the I2C device cannot be opened or the
    /// PCA9685 fails to initialize.
    pub fn new(i2c_bus: &str, address: u8) -> Result<Self, MotorError> {
        let dev = I2cdev::new(i2c_bus)
            .map_err(|e| MotorError::Init(format!("failed to open I2C bus {i2c_bus}: {e}")))?;
        let mut pwm = Pca9685::new(dev, address).map_err(|e| {
            MotorError::Init(format!("failed to create PCA9685 at {address:#04x}: {e:?}"))
        })?;
        pwm.set_prescale(100)
            .map_err(|e| MotorError::Pwm(format!("failed to set prescale: {e:?}")))?;
        pwm.enable()
            .map_err(|e| MotorError::Pwm(format!("failed to enable PCA9685: {e:?}")))?;

        Ok(Self {
            pwm: std::sync::Mutex::new(pwm),
        })
    }
}

impl Drop for Pca9685MotorBoard {
    fn drop(&mut self) {
        // Best-effort: stop all motors when the board is dropped.
        let _ = self.stop_all();
    }
}

impl MotorDriver for Pca9685MotorBoard {
    fn set_motor_speed(&self, motor: Motor, speed: u16, forward: bool) -> Result<(), MotorError> {
        let channels = channels_for(motor);
        let mut pwm = self
            .pwm
            .lock()
            .map_err(|e| MotorError::Pwm(format!("mutex poisoned: {e}")))?;

        // Reference channel always high
        pwm.set_channel_on(channels.ref_channel, 0)
            .map_err(|e| MotorError::Pwm(format!("{e:?}")))?;
        pwm.set_channel_off(channels.ref_channel, 4095)
            .map_err(|e| MotorError::Pwm(format!("{e:?}")))?;

        if forward {
            pwm.set_channel_on(channels.forward_channel, 0)
                .map_err(|e| MotorError::Pwm(format!("{e:?}")))?;
            pwm.set_channel_off(channels.forward_channel, speed)
                .map_err(|e| MotorError::Pwm(format!("{e:?}")))?;
            pwm.set_channel_on(channels.backward_channel, 0)
                .map_err(|e| MotorError::Pwm(format!("{e:?}")))?;
            pwm.set_channel_off(channels.backward_channel, 0)
                .map_err(|e| MotorError::Pwm(format!("{e:?}")))?;
        } else {
            pwm.set_channel_on(channels.forward_channel, 0)
                .map_err(|e| MotorError::Pwm(format!("{e:?}")))?;
            pwm.set_channel_off(channels.forward_channel, 0)
                .map_err(|e| MotorError::Pwm(format!("{e:?}")))?;
            pwm.set_channel_on(channels.backward_channel, 0)
                .map_err(|e| MotorError::Pwm(format!("{e:?}")))?;
            pwm.set_channel_off(channels.backward_channel, speed)
                .map_err(|e| MotorError::Pwm(format!("{e:?}")))?;
        }

        Ok(())
    }

    /// Attempt all six zero-output writes, collecting motor/channel/operation
    /// diagnostics if any fail. Failure leaves the actual output uncertain.
    fn stop_motor(&self, motor: Motor) -> Result<(), MotorError> {
        let mut pwm = self.pwm.lock().map_err(|e| {
            MotorError::Pwm(format!("{motor:?} stop_motor lock: mutex poisoned: {e}"))
        })?;

        stop_motor_outputs(motor, |channel, operation, value| match operation {
            StopWrite::On => pwm.set_channel_on(channel, value),
            StopWrite::Off => pwm.set_channel_off(channel, value),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn motor_stop_attempts_all_writes_and_collects_failures() {
        // Explicit expected maps protect the existing electrical order instead
        // of deriving expectations from the production channel mapping.
        for (motor, channels) in [
            (Motor::Motor1, [Channel::C8, Channel::C9, Channel::C10]),
            (Motor::Motor2, [Channel::C13, Channel::C11, Channel::C12]),
            (Motor::Motor3, [Channel::C2, Channel::C3, Channel::C4]),
            (Motor::Motor4, [Channel::C7, Channel::C5, Channel::C6]),
        ] {
            let expected = [
                (channels[0], StopWrite::On, 0),
                (channels[0], StopWrite::Off, 0),
                (channels[1], StopWrite::On, 0),
                (channels[1], StopWrite::Off, 0),
                (channels[2], StopWrite::On, 0),
                (channels[2], StopWrite::Off, 0),
            ];
            // Every subset includes first/middle/final write failures, combined
            // failures, a completely failing bus and the successful sequence.
            for failures in 0_u8..64 {
                let mut attempts = Vec::new();
                let result = stop_motor_outputs(motor, |channel, operation, value| {
                    let index = attempts.len();
                    attempts.push((channel, operation, value));
                    if failures & (1 << index) == 0 {
                        Ok(())
                    } else {
                        Err(format!("injected failure {index}"))
                    }
                });
                assert_eq!(attempts, expected);
                if failures == 0 {
                    assert!(result.is_ok());
                    continue;
                }
                let Err(MotorError::Pwm(message)) = result else {
                    panic!("expected stop failure for {motor:?}, mask {failures}");
                };
                for (index, (channel, _, _)) in expected.iter().enumerate() {
                    let operation = if index % 2 == 0 {
                        "set_channel_on"
                    } else {
                        "set_channel_off"
                    };
                    let diagnostic = format!(
                        "{motor:?} channel {channel:?} {operation}(0): \"injected failure {index}\""
                    );
                    assert_eq!(message.contains(&diagnostic), failures & (1 << index) != 0);
                }
                assert_eq!(
                    message.matches("injected failure").count(),
                    failures.count_ones() as usize
                );
            }
        }
    }
}
