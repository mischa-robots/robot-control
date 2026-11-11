//! Exercise the public API with a downstream-style custom driver.
use std::sync::{Arc, Mutex as StdMutex};
use std::time::Duration;

use robot_control::{Motor, MotorDriver, MotorError, Robot, RobotConfig, RobotError};
use tokio::sync::Mutex;

#[derive(Clone, Default)]
struct RecordingDriver(Arc<StdMutex<Vec<(Motor, u16, bool)>>>);

impl MotorDriver for RecordingDriver {
    fn set_motor_speed(&self, motor: Motor, speed: u16, forward: bool) -> Result<(), MotorError> {
        self.0.lock().unwrap().push((motor, speed, forward));
        Ok(())
    }

    fn stop_motor(&self, motor: Motor) -> Result<(), MotorError> {
        self.0.lock().unwrap().push((motor, 0, false));
        Ok(())
    }
}

#[tokio::test(start_paused = true)]
async fn saturated_commands_reach_driver_as_bounded_pwm() {
    let driver = RecordingDriver::default();
    let log = driver.0.clone();
    // Explicit literal ensures no mandatory public fields have been added.
    let config = RobotConfig {
        left_motor: Motor::Motor4,
        right_motor: Motor::Motor1,
        left_motor_factor: f32::MAX,
        right_motor_factor: -f32::MAX,
        adjustment_step: 1.0,
        adjustment_interval: Duration::from_millis(10),
    };
    let robot = Robot::new(Arc::new(Mutex::new(driver)), config).unwrap();
    for (demand, left_forward, right_forward) in [(f32::MAX, true, false), (-f32::MAX, false, true)]
    {
        robot.drive(demand, demand).await.unwrap();
        // Two ticks also allow a full +1 to -1 transition at step 1.
        for _ in 0..2 {
            tokio::task::yield_now().await;
            tokio::time::advance(Duration::from_millis(10)).await;
            tokio::task::yield_now().await;
        }
        let commands = log.lock().unwrap().clone();
        assert!(commands.iter().all(|(_, pwm, _)| *pwm <= 4095));
        assert_eq!(
            &commands[commands.len() - 2..],
            &[
                (Motor::Motor4, 4095, left_forward),
                (Motor::Motor1, 4095, right_forward)
            ]
        );
    }
    robot.drive(0.0, 0.0).await.unwrap();
    tokio::time::advance(Duration::from_millis(10)).await;
    tokio::task::yield_now().await;
    let commands = log.lock().unwrap().clone();
    assert_eq!(
        &commands[commands.len() - 2..],
        &[(Motor::Motor4, 0, false), (Motor::Motor1, 0, false)]
    );
    robot.shutdown().await;
}

// Compile exhaustive matches to preserve the existing error enum surface.
fn existing_error_api(error: RobotError) -> String {
    match error {
        RobotError::Shutdown => "shutdown".into(),
        RobotError::Motor(error) => match error {
            MotorError::Init(message) | MotorError::Pwm(message) | MotorError::I2c(message) => {
                message
            }
        },
    }
}

#[tokio::test]
async fn invalid_input_reports_through_existing_error_api() {
    let robot = Robot::new(
        Arc::new(Mutex::new(RecordingDriver::default())),
        RobotConfig::default(),
    )
    .unwrap();
    let message = existing_error_api(robot.drive(0.5, f32::NAN).await.unwrap_err());
    assert!(message.contains("finite"));
    robot.shutdown().await;
}
