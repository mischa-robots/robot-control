//! Bounded fault/status demonstration using only a mock driver. No hardware I/O.
use std::{sync::Arc, time::Duration};

use clap::Parser;
use robot_control::{
    Motor, MotorDriver, MotorError, Robot, RobotConfig, RobotOutputState, RobotPhase,
};
use tokio::sync::{Mutex, Notify};

#[derive(Parser)]
#[command(
    version,
    about = "Demonstrate latched controller faults without hardware"
)]
struct Cli {
    /// Also inject a failure while stopping M1 during cleanup.
    #[arg(long)]
    cleanup_failure: bool,
}

struct FailingDriver {
    failed: Arc<Notify>,
    cleanup_failure: bool,
}

impl MotorDriver for FailingDriver {
    fn set_motor_speed(&self, motor: Motor, speed: u16, forward: bool) -> Result<(), MotorError> {
        println!("Mock write: {motor:?}, PWM={speed}, forward={forward}");
        self.failed.notify_one();
        Err(MotorError::I2c("injected output failure".into()))
    }

    fn stop_motor(&self, motor: Motor) -> Result<(), MotorError> {
        println!("Mock stop attempted: {motor:?}");
        if self.cleanup_failure && motor == Motor::Motor1 {
            Err(MotorError::I2c("injected cleanup failure".into()))
        } else {
            Ok(())
        }
    }
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();
    let failed = Arc::new(Notify::new());
    let robot = Robot::new(
        Arc::new(Mutex::new(FailingDriver {
            failed: failed.clone(),
            cleanup_failure: cli.cleanup_failure,
        })),
        RobotConfig {
            left_motor: Motor::Motor4,
            right_motor: Motor::Motor1,
            ..RobotConfig::default()
        },
    )?;
    println!("Initial status: {:?}", robot.status().await);
    robot.drive(0.25, 0.25).await?;
    println!("Target accepted; hardware application has not been confirmed.");
    let result = tokio::time::timeout(Duration::from_secs(2), failed.notified()).await;
    // Always join cleanup, including when the demonstration's deadline elapses.
    robot.shutdown().await;
    result?;
    let status = robot.status().await;
    println!("After cleanup: {status:?}");
    assert_eq!(status.phase, RobotPhase::Faulted);
    assert!(status.cleanup_complete);
    assert!(
        status
            .fault
            .as_ref()
            .unwrap()
            .contains("injected output failure")
    );
    assert_eq!(status.cleanup_error.is_some(), cli.cleanup_failure);
    assert_eq!(
        status.output,
        if cli.cleanup_failure {
            RobotOutputState::Unknown
        } else {
            RobotOutputState::Stopped
        }
    );
    let error = robot.drive(0.25, 0.25).await.unwrap_err();
    println!("Expected rejected drive: {error}");
    robot.shutdown().await;
    assert_eq!(robot.status().await, status);
    println!("Fault remains latched after repeated shutdown; recovery needs a fresh Robot.");
    Ok(())
}
