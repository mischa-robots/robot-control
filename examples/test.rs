//! Test M1–M4 one at a time over I2C.
//!
//! Run with: cargo run --example test --features pca9685

#[cfg(feature = "pca9685")]
use clap::Parser;
#[cfg(feature = "pca9685")]
use robot_control::{Motor, MotorDriver, Pca9685MotorBoard};
#[cfg(feature = "pca9685")]
use std::{thread::sleep, time::Duration};

#[cfg(feature = "pca9685")]
#[derive(Parser)]
#[command(about = "Test each motor: forward, zero, reverse, stop")]
struct Cli {
    #[arg(long, default_value = "/dev/i2c-1")]
    i2c_bus: String,

    /// I2C address in decimal (96 = 0x60).
    #[arg(long, default_value_t = 0x60)]
    i2c_addr: u8,
}

/// Ramp from rest to full speed over 100 ms, then hold for one second.
#[cfg(feature = "pca9685")]
fn ramp_and_hold(
    board: &Pca9685MotorBoard,
    motor: Motor,
    forward: bool,
) -> Result<(), robot_control::MotorError> {
    for step in 1_u16..=10 {
        sleep(Duration::from_millis(10));
        board.set_motor_speed(motor, 4095 * step / 10, forward)?;
    }
    sleep(Duration::from_secs(1));
    Ok(())
}

#[cfg(feature = "pca9685")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();
    let board = Pca9685MotorBoard::new(&cli.i2c_bus, cli.i2c_addr)?;
    board.stop_all()?;

    let motors = [Motor::Motor1, Motor::Motor2, Motor::Motor3, Motor::Motor4];
    for (index, motor) in motors.into_iter().enumerate() {
        let number = index + 1;
        println!("M{number}: ramping to full forward (1), holding for 1 second");
        ramp_and_hold(&board, motor, true)?;

        println!("M{number}: zero (0) for 1 second");
        board.stop_motor(motor)?;
        sleep(Duration::from_secs(1));

        println!("M{number}: ramping to full reverse (-1), holding for 1 second");
        ramp_and_hold(&board, motor, false)?;

        board.stop_motor(motor)?;
        println!("M{number}: stopped");
        if index < motors.len() - 1 {
            println!("Pausing for 2 seconds");
            sleep(Duration::from_secs(2));
        }
    }

    println!("Motor test complete");
    Ok(())
}

#[cfg(not(feature = "pca9685"))]
fn main() {
    eprintln!("Run with: cargo run --example test --features pca9685");
    std::process::exit(1);
}
