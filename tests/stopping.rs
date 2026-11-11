//! Test the default `stop_all` through an existing downstream-style driver.
use std::sync::Mutex;

use robot_control::{Motor, MotorDriver, MotorError};

const MOTORS: [Motor; 4] = [Motor::Motor1, Motor::Motor2, Motor::Motor3, Motor::Motor4];

struct FailingStops {
    failures: u8,
    attempts: Mutex<Vec<Motor>>,
}

impl MotorDriver for FailingStops {
    fn set_motor_speed(&self, _: Motor, _: u16, _: bool) -> Result<(), MotorError> {
        panic!("stopping must never set a drive speed");
    }

    fn stop_motor(&self, motor: Motor) -> Result<(), MotorError> {
        self.attempts.lock().unwrap().push(motor);
        let index = MOTORS
            .iter()
            .position(|candidate| *candidate == motor)
            .unwrap();
        if self.failures & (1 << index) == 0 {
            return Ok(());
        }
        // Existing driver error variants must all survive aggregation as text.
        Err(match motor {
            Motor::Motor1 => MotorError::I2c("bus unavailable".into()),
            Motor::Motor2 => MotorError::Pwm("write rejected".into()),
            Motor::Motor3 => MotorError::Init("driver unavailable".into()),
            Motor::Motor4 => MotorError::I2c("final stop failed".into()),
        })
    }
}

#[test]
fn stop_all_attempts_every_motor_and_reports_every_failure() {
    let diagnostics = [
        "Motor1 stop_motor: I2C error: bus unavailable",
        "Motor2 stop_motor: PWM error: write rejected",
        "Motor3 stop_motor: initialization failed: driver unavailable",
        "Motor4 stop_motor: I2C error: final stop failed",
    ];
    // All subsets cover success, each individual failure and multiple failures.
    for failures in 0..16 {
        let driver = FailingStops {
            failures,
            attempts: Mutex::new(Vec::new()),
        };
        let result = driver.stop_all();
        assert_eq!(*driver.attempts.lock().unwrap(), MOTORS);
        if failures == 0 {
            assert!(result.is_ok());
            continue;
        }
        let Err(MotorError::Pwm(message)) = result else {
            panic!("expected collected stop failure for mask {failures}");
        };
        for (index, diagnostic) in diagnostics.iter().enumerate() {
            assert_eq!(message.contains(diagnostic), failures & (1 << index) != 0);
        }
        assert_eq!(
            message.matches("stop_motor:").count(),
            failures.count_ones() as usize
        );
    }
}
