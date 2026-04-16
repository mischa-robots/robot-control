# Robot Control

[![Crates.io](https://img.shields.io/crates/v/robot-control.svg)](https://crates.io/crates/robot-control)
[![Docs.rs](https://docs.rs/robot-control/badge.svg)](https://docs.rs/robot-control)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

A Rust library for controlling DC motor robots via PCA9685 PWM boards over I2C.

## Features

- **Hardware-abstracted `MotorDriver` trait** — swap motor boards or use a mock for testing
- **Smooth motor ramping** — gradual acceleration/deceleration prevents mechanical stress
- **Configurable** — motor factors, adjustment step, ramp interval via `RobotConfig`
- **Testable** — full unit test suite runs without any hardware
- **Web server example included** — browser-based dual-joystick UI with WebSocket and HTTP control

## Hardware Requirements

- A single-board computer with I2C support (Jetson Nano, Raspberry Pi, etc.)
- A **PCA9685**-based motor driver board (e.g., Adafruit DC Motor + Stepper FeatherWing)
- DC motors connected to motor ports 1–4
- **Stepper motors are not supported, yet.**

## Installation

### As a library

```toml
[dependencies]
robot-control = "0.1"
```

For real hardware support, enable the `pca9685` feature:

```toml
[dependencies]
robot-control = { version = "0.1", features = ["pca9685"] }
```

### Running the web server example

```bash
# Clone the repo to get the example and static/ files
git clone https://github.com/mischa-robots/robot-control.git
cd robot-control
cargo run --example web_server --features pca9685
```

## Usage

### Library API

```rust,no_run
use robot_control::{Motor, MotorDriver, Robot, RobotConfig};
use std::sync::Arc;
use tokio::sync::Mutex;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // With real hardware (requires `pca9685` feature):
    // let board = robot_control::Pca9685MotorBoard::new("/dev/i2c-1", 0x60)?;
    // let driver: Arc<Mutex<dyn MotorDriver>> = Arc::new(Mutex::new(board));

    // Configure motor factors (use -1.0 to reverse a motor's polarity)
    let config = RobotConfig {
        left_motor_factor: -1.0,
        right_motor_factor: 1.0,
        ..RobotConfig::default()
    };

    // let robot = Robot::new(driver, config)?;
    // robot.drive(0.5, 0.5).await?;   // drive forward at 50% speed
    // robot.stop().await?;              // ramp down to zero
    // robot.shutdown().await;            // stop motors and cancel background task
    Ok(())
}
```

### Web Server Example

The repository includes a full web server example (`examples/web_server.rs`) with a dual-joystick UI:

```bash
# Start with default settings (binds to 0.0.0.0:8000)
cargo run --example web_server --features pca9685

# Custom configuration
cargo run --example web_server --features pca9685 -- \
  --host 0.0.0.0 \
  --port 8080 \
  --i2c-bus /dev/i2c-1 \
  --i2c-addr 96 \
  --left-factor -1.0 \
  --right-factor 1.0
```

Then open `http://<robot-ip>:8000` in a browser for the joystick control UI.

#### Endpoints (web server example)

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Web UI with dual joystick control |
| `/ws` | WebSocket | Real-time drive commands (`{"left": 0.5, "right": 0.5}`) |
| `/drive?left=0.5&right=0.5` | GET | One-shot drive command |
| `/health` | GET | Health check |

## Architecture

```
┌─────────────────────────────────────────────────┐
│  Browser (joystick UI)                          │
│  └─ WebSocket ──────────────────┐               │
│                                 │               │
│  ┌──────────────────────────────▼─────────────┐ │
│  │  Web Server (examples/web_server.rs)       │ │
│  │  ├─ /ws → WebSocket actor                  │ │
│  │  ├─ /drive → HTTP handler                  │ │
│  │  └─ / → static files                       │ │
│  └──────────────────────────────┬─────────────┘ │
│                                 │               │
│  ┌──────────────────────────────▼─────────────┐ │
│  │  Robot (robot-control library)             │ │
│  │  ├─ Smooth motor ramping (background task) │ │
│  │  ├─ Input clamping [-1.0, 1.0]             │ │
│  │  └─ Motor factor application               │ │
│  └──────────────────────────────┬─────────────┘ │
│                                 │               │
│  ┌──────────────────────────────▼─────────────┐ │
│  │  MotorDriver trait                         │ │
│  │  ├─ Pca9685MotorBoard (feature: pca9685)   │ │
│  │  └─ MockMotorDriver (for testing)          │ │
│  └────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────┘
```

## Feature Flags

| Flag | Default | Description |
|------|---------|-------------|
| `pca9685` | off | Enables `Pca9685MotorBoard` with real I2C hardware support via `linux-embedded-hal` and `pwm-pca9685` |

## Development

```bash
# Run tests (no hardware needed)
cargo test

# Check formatting + lints
cargo fmt --all --check
cargo clippy --all-targets -- -D warnings

# Build for real hardware
cargo build --release --features pca9685

# Build docs
cargo doc --no-deps --open
```

## Contributing

Contributions are welcome! Please ensure `cargo fmt`, `cargo clippy`, and `cargo test` all pass before submitting a PR.

## License

This project is licensed under the [MIT License](LICENSE).

Copyright (c) 2026 Michael Schaefer <https://github.com/mischa-robots/robot-control>
