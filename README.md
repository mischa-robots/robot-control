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

### Validation (0.1.2)

`Robot::new` rejects duplicate left/right motors, zero adjustment intervals, non-positive or non-finite adjustment steps, non-finite factors, and a step/interval ratio that is zero or non-finite in `f32`. Validation happens before the controller task starts; constructing a hardware driver separately can already access the board.

`drive` rejects NaN or infinity in either input without replacing either target. Finite input is clamped to `[-1, 1]`, multiplied by its factor, then clamped again. Factors above one still work but saturate at supported power; negative factors reverse polarity and zero disables that motor's demand. For example, input `0.75` with factor `2` produces `1.0`, while input `2` with factor `0.5` produces `0.5`.

The public API and defaults are unchanged. To preserve existing error enum matches, configuration errors use `RobotError::Motor(MotorError::Init(...))` and invalid drive inputs use `RobotError::Motor(MotorError::Pwm(...))`, with validation details in the message. These errors do not imply an I2C transaction was attempted.

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

#### Architecture (web server example)

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

### Motor Test Example

`examples/test.rs` tests M1–M4 one at a time, ramping each to full speed forward and reverse with a stop between directions. Requires PCA9685 hardware; defaults to `/dev/i2c-1` at address `0x60` (override with `--i2c-bus` and `--i2c-addr`, using a decimal address).

```bash
cargo run --example test --features pca9685
```

### Controller verification on hardware (0.1.2)

`controller_check` exercises `Robot`, unlike the raw M1–M4 motor test. It defaults to left M1/right M4, factors +1, and power 0.25. Keep the tracks clear of the surface, use a power that moves your motors safely, and keep a power cutoff accessible. Stop any other process controlling the board first.

```bash
cargo run --example controller_check --features pca9685 -- --power 0.25
# Check existing left polarity behavior in a second run:
cargo run --example controller_check --features pca9685 -- --power 0.25 --left-factor -1.0
```

The bounded sequence prints each stage: zero, left-only, zero, right-only, forward, zero, reverse, zero (one second each, including ramp). It then submits a valid left demand with a NaN right demand. Expect an `Expected invalid-input rejection` message and both tracks to remain at zero for another second, followed by explicit shutdown. This does not measure wheel speed or confirm hardware output through sensors. Ctrl-C/SIGTERM also requests shutdown; existing shutdown limitations are unchanged in this patch.

Check configuration rejection separately:

```bash
cargo run --example controller_check --features pca9685 -- --right-motor 1
cargo run --example controller_check --features pca9685 -- --interval-ms 0
```

Both should print `Controller rejected configuration`, exit with an error and never begin the motion sequence. The example opens and stops the board before creating the controller, so these are not tests of hardware-free startup. Use `--i2c-bus /dev/i2c-1 --i2c-addr 96` to override the connection and `--left-motor`/`--right-motor` to match your wiring. Avoid large factors on hardware: they can saturate at full power; mock tests cover those extremes.

Compare directions and the unchanged baseline sequence with 0.1.1. Record wiring, factors, requested power and results. Unexpected movement or continuing output after shutdown should end the test using the power cutoff. Automated tests verify rejection before task startup, preservation of both targets, and bounded PWM output without connecting hardware.


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
