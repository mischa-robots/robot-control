//! Bounded, operator-run checks through Robot (not direct motor commands).
use std::{
    sync::Arc,
    time::{Duration, Instant},
};

use clap::{Parser, ValueEnum};
use robot_control::{
    Motor, MotorDriver, MotorError, Pca9685MotorBoard, Robot, RobotConfig, RobotError,
};
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
    /// Adjustment interval in milliseconds (10..=20 for this example).
    #[arg(long, default_value_t = 10)]
    interval_ms: u64,
    /// Nominal power change per interval (0.01..=0.1 for this example).
    #[arg(long, default_value_t = 0.1, allow_hyphen_values = true)]
    adjustment_step: f32,
    /// Print timestamped output writes (PWM demand, not measured wheel speed).
    #[arg(long)]
    trace_output: bool,
    /// Finite requested power in (0, 1]; factors are applied afterward.
    #[arg(long, default_value_t = 0.25)]
    power: f32,
    /// Append alternating forward/pivot commands to check paired target changes.
    #[arg(long)]
    paired_targets: bool,
    /// After the baseline, check shutdown from motion or last-handle drop.
    #[arg(long, value_enum)]
    lifecycle_check: Option<LifecycleCheck>,
}

#[derive(Clone, Copy, ValueEnum)]
enum LifecycleCheck {
    Forward,
    Reverse,
    Drop,
}

struct TracedBoard {
    board: Pca9685MotorBoard,
    started: Instant,
}

impl MotorDriver for TracedBoard {
    fn set_motor_speed(&self, motor: Motor, speed: u16, forward: bool) -> Result<(), MotorError> {
        let result = self.board.set_motor_speed(motor, speed, forward);
        println!(
            "output t={:.6}s motor={motor:?} pwm={speed} forward={forward} ok={}",
            self.started.elapsed().as_secs_f64(),
            result.is_ok()
        );
        result
    }

    fn stop_motor(&self, motor: Motor) -> Result<(), MotorError> {
        let result = self.board.stop_motor(motor);
        println!(
            "output t={:.6}s motor={motor:?} pwm=0 stop ok={}",
            self.started.elapsed().as_secs_f64(),
            result.is_ok()
        );
        result
    }
}

fn motor(number: u8) -> Motor {
    match number {
        1 => Motor::Motor1,
        2 => Motor::Motor2,
        3 => Motor::Motor3,
        _ => Motor::Motor4, // CLI restricts the value to 1..=4.
    }
}

async fn sequence(
    robot: &Robot,
    power: f32,
    paired_targets: bool,
    started: Instant,
) -> Result<(), RobotError> {
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
        println!(
            "{name}: left={left}, right={right} at t={:.6}s (one second including ramp)",
            started.elapsed().as_secs_f64()
        );
        robot.drive(left, right).await?;
        tokio::time::sleep(Duration::from_secs(1)).await;
        println!("Controller status: {:?}", robot.status().await);
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
                "paired targets / {name}: left={left}, right={right} at t={:.6}s (one second including ramp)",
                started.elapsed().as_secs_f64()
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

async fn lifecycle_check(
    robot: Arc<Robot>,
    power: f32,
    check: LifecycleCheck,
    started: Instant,
) -> Result<(), Box<dyn std::error::Error>> {
    let demand = if matches!(check, LifecycleCheck::Reverse) {
        -power
    } else {
        power
    };
    println!(
        "Lifecycle check: left={demand}, right={demand} at t={:.6}s for one second including ramp",
        started.elapsed().as_secs_f64()
    );
    let result: Result<bool, Box<dyn std::error::Error>> = tokio::select! {
        result = async {
            robot.drive(demand, demand).await?;
            tokio::time::sleep(Duration::from_secs(1)).await;
            Ok::<_, RobotError>(true)
        } => result.map_err(Into::into),
        result = interrupted() => result.map(|()| false).map_err(Into::into),
    };
    if matches!(result, Ok(true)) && matches!(check, LifecycleCheck::Drop) {
        // Consume the only public handle. The caller retains the motor driver,
        // proving this check does not depend on the board itself being dropped.
        drop(robot);
        println!(
            "Last Robot handle dropped; observing best-effort cleanup for two seconds with the runtime alive."
        );
    } else {
        // Also await cleanup on an error or signal during the motion stage.
        robot.shutdown().await;
        robot.shutdown().await;
        println!("Final controller status: {:?}", robot.status().await);
        println!(
            "Shutdown completed twice; observing outputs for two seconds. Check driver warnings."
        );
    }
    tokio::time::sleep(Duration::from_secs(2)).await;
    println!("Observation window finished; confirm outputs stayed off.");
    result.map(|_| ())
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();
    if !cli.power.is_finite() || cli.power <= 0.0 || cli.power > 1.0 {
        return Err("--power must be finite and in (0, 1]".into());
    }
    if !(0.01..=0.1).contains(&cli.adjustment_step) {
        return Err("--adjustment-step must be finite and in [0.01, 0.1] for this example".into());
    }
    if !(10..=20).contains(&cli.interval_ms) {
        return Err("--interval-ms must be in [10, 20] for this example".into());
    }
    tracing_subscriber::fmt::init();
    let board = Pca9685MotorBoard::new(&cli.i2c_bus, cli.i2c_addr)?;
    board.stop_all()?;
    let started = Instant::now();
    let driver: Arc<Mutex<dyn MotorDriver>> = if cli.trace_output {
        Arc::new(Mutex::new(TracedBoard { board, started }))
    } else {
        Arc::new(Mutex::new(board))
    };
    let config = RobotConfig {
        left_motor: motor(cli.left_motor),
        right_motor: motor(cli.right_motor),
        left_motor_factor: cli.left_factor,
        right_motor_factor: cli.right_factor,
        adjustment_interval: Duration::from_millis(cli.interval_ms),
        adjustment_step: cli.adjustment_step,
    };
    let robot = match Robot::new(driver.clone(), config) {
        Ok(robot) => robot,
        Err(error) => {
            eprintln!("Controller rejected configuration: {error}");
            driver.lock().await.stop_all()?;
            return Err(error.into());
        }
    };
    println!("Initial controller status: {:?}", robot.status().await);
    let result: Result<bool, Box<dyn std::error::Error>> = tokio::select! {
        result = sequence(&robot, cli.power, cli.paired_targets, started) => result.map(|()| true).map_err(Into::into),
        result = interrupted() => result.map(|()| false).map_err(Into::into),
    };
    // A signal ending the baseline must not start a new motion check.
    if matches!(result, Ok(true)) {
        if let Some(check) = cli.lifecycle_check {
            return lifecycle_check(robot, cli.power, check, started).await;
        }
    }
    robot.shutdown().await;
    println!("Final controller status: {:?}", robot.status().await);
    println!("Shutdown completed; inspect the robot and any driver warnings.");
    result.map(|_| ())
}
