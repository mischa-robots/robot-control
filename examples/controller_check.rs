//! Bounded, operator-run checks through Robot (not direct motor commands).
use std::{sync::Arc, time::Duration};

use clap::Parser;
use robot_control::{Motor, MotorDriver, Pca9685MotorBoard, Robot, RobotConfig, RobotError};
use tokio::sync::Mutex;

#[derive(Parser)]
#[command(
    version,
    about = "Check controller validation and a bounded two-motor sequence"
)]
struct Cli {
    #[arg(long, default_value = "/dev/i2c-1")]
    i2c_bus: String,
    /// Decimal address (96 = 0x60).
    #[arg(long, default_value_t = 96)]
    i2c_addr: u8,
    #[arg(long, default_value_t = 4, value_parser = clap::value_parser!(u8).range(1..=4))]
    left_motor: u8,
    #[arg(long, default_value_t = 1, value_parser = clap::value_parser!(u8).range(1..=4))]
    right_motor: u8,
    #[arg(long, default_value_t = 1.0, allow_hyphen_values = true)]
    left_factor: f32,
    #[arg(long, default_value_t = 1.0, allow_hyphen_values = true)]
    right_factor: f32,
    #[arg(long, default_value_t = 10)]
    interval_ms: u64,
    /// Finite requested power in (0, 1]; factors are applied afterward.
    #[arg(long, default_value_t = 0.25)]
    power: f32,
    /// Append alternating forward/pivot commands to check paired target changes.
    #[arg(long)]
    paired_targets: bool,
}

fn motor(number: u8) -> Motor {
    match number {
        1 => Motor::Motor1,
        2 => Motor::Motor2,
        3 => Motor::Motor3,
        _ => Motor::Motor4, // CLI restricts the value to 1..=4.
    }
}

async fn sequence(robot: &Robot, power: f32, paired_targets: bool) -> Result<(), RobotError> {
    for (name, left, right) in [
        ("zero", 0.0, 0.0),
        ("left only", power, 0.0),
        ("zero", 0.0, 0.0),
        ("right only", 0.0, power),
        ("forward", power, power),
        ("zero", 0.0, 0.0),
        ("reverse", -power, -power),
        ("zero", 0.0, 0.0),
    ] {
        println!("{name}: left={left}, right={right} (one second including ramp)");
        robot.drive(left, right).await?;
        tokio::time::sleep(Duration::from_secs(1)).await;
    }
    if paired_targets {
        for (name, left, right) in [
            ("forward", power, power),
            ("pivot right", power, -power),
            ("forward", power, power),
            ("pivot left", -power, power),
            ("forward", power, power),
            ("zero", 0.0, 0.0),
        ] {
            println!(
                "paired targets / {name}: left={left}, right={right} (one second including ramp)"
            );
            robot.drive(left, right).await?;
            tokio::time::sleep(Duration::from_secs(1)).await;
        }
    }
    // The valid left input must not replace the preceding zero target when the
    // right input is invalid. Hardware should remain at zero throughout.
    match robot.drive(power, f32::NAN).await {
        Err(error) => println!("Expected invalid-input rejection: {error}"),
        Ok(()) => {
            return Err(
                robot_control::MotorError::Pwm("invalid command was accepted".into()).into(),
            );
        }
    }
    tokio::time::sleep(Duration::from_secs(1)).await;
    robot.stop().await
}

async fn interrupted() -> std::io::Result<()> {
    #[cfg(unix)]
    {
        let mut terminate =
            tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
        tokio::select! {
            result = tokio::signal::ctrl_c() => result,
            _ = terminate.recv() => Ok(()),
        }
    }
    #[cfg(not(unix))]
    tokio::signal::ctrl_c().await
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();
    if !cli.power.is_finite() || cli.power <= 0.0 || cli.power > 1.0 {
        return Err("--power must be finite and in (0, 1]".into());
    }
    tracing_subscriber::fmt::init();
    let board = Pca9685MotorBoard::new(&cli.i2c_bus, cli.i2c_addr)?;
    board.stop_all()?;
    let driver: Arc<Mutex<dyn MotorDriver>> = Arc::new(Mutex::new(board));
    let config = RobotConfig {
        left_motor: motor(cli.left_motor),
        right_motor: motor(cli.right_motor),
        left_motor_factor: cli.left_factor,
        right_motor_factor: cli.right_factor,
        adjustment_interval: Duration::from_millis(cli.interval_ms),
        ..RobotConfig::default()
    };
    let robot = match Robot::new(driver.clone(), config) {
        Ok(robot) => robot,
        Err(error) => {
            eprintln!("Controller rejected configuration: {error}");
            driver.lock().await.stop_all()?;
            return Err(error.into());
        }
    };
    let result: Result<(), Box<dyn std::error::Error>> = tokio::select! {
        result = sequence(&robot, cli.power, cli.paired_targets) => result.map_err(Into::into),
        result = interrupted() => result.map_err(Into::into),
    };
    robot.shutdown().await;
    println!("Shutdown requested; inspect the robot and any driver warnings.");
    result
}
