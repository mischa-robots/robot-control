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
    let board = robot_control::Pca9685MotorBoard::new("/dev/i2c-1", 0x60)?;
    let driver: Arc<Mutex<dyn MotorDriver>> = Arc::new(Mutex::new(board));

    // Configure motor factors (use -1.0 to reverse a motor's polarity)
    let config = RobotConfig {
        left_motor: Motor::Motor4,
        right_motor: Motor::Motor1,
        left_motor_factor: -1.0,
        right_motor_factor: 1.0,
        ..RobotConfig::default()
    };

    let robot = Robot::new(driver, config)?;
    robot.drive(0.5, 0.5).await?;   // drive forward at 50% speed
    robot.stop().await?;              // ramp down to zero
    robot.shutdown().await;            // join adjustment, then finish the stop attempt
    
    Ok(())
}
```

### Validation (0.1.2)

`Robot::new` rejects duplicate left/right motors, zero adjustment intervals, non-positive or non-finite adjustment steps, non-finite factors, and a step/interval ratio that is zero or non-finite in `f32`. Validation happens before the controller task starts; constructing a hardware driver separately can already access the board.

`drive` rejects NaN or infinity in either input without replacing either target. Finite input is clamped to `[-1, 1]`, multiplied by its factor, then clamped again. Factors above one still work but saturate at supported power; negative factors reverse polarity and zero disables that motor's demand. For example, input `0.75` with factor `2` produces `1.0`, while input `2` with factor `0.5` produces `0.5`.

The public API and defaults are unchanged. To preserve existing error enum matches, configuration errors use `RobotError::Motor(MotorError::Init(...))` and invalid drive inputs use `RobotError::Motor(MotorError::Pwm(...))`, with validation details in the message. These errors do not imply an I2C transaction was attempted.

### Paired targets (0.1.3)

`drive(left, right)` accepts both targets atomically, and `stop()` atomically replaces the pair with zero. Each adjustment tick reads one complete pair, even when commands run concurrently. Commands waiting behind shutdown recheck admission under the same lock and return `RobotError::Shutdown` without changing targets once cancellation has begun.

Successful `drive` means target acceptance. Each motor still ramps independently, and the driver writes the left and right outputs sequentially. Paired acceptance does not change ramp timing, polarity, PWM mapping or electrical stopping.

### Exhaustive stopping (0.1.4)

The default `MotorDriver::stop_all()` attempts M1–M4 in order even if an earlier stop fails. `Pca9685MotorBoard::stop_motor()` similarly attempts all six writes: reference-channel on/off, forward-channel on/off, then backward-channel on/off, all with value zero. The healthy-board write sequence and electrical behavior are unchanged.

Failures are collected into the existing `MotorError::Pwm` variant. Messages identify every failed motor and retain its diagnostic; PCA9685 write failures also identify the channel and operation. A returned error means actual output is uncertain even though the remaining stops were attempted. Custom drivers overriding `stop_all()` must implement their own exhaustive stopping behavior.

`Robot::stop()` still accepts a smooth ramp to zero; it does not report subsequent hardware failures. `Robot::shutdown()` logs stop failures and retains its existing `()` return type. Exhaustive stopping does not add a fault-status API.

### Ordered shutdown and task ownership (0.1.5)

`robot.shutdown().await` closes command admission under the paired target lock, requests cancellation, waits for the adjustment task to terminate, and then finishes the final `stop_all()` attempt. New `drive` and `stop` calls return `RobotError::Shutdown` once admission closes. No adjustment writes occur after shutdown returns. In-flight driver I/O must finish first; shutdown does not abort it or wait for a smooth ramp to zero.

Repeated and concurrent shutdown calls wait for the same cleanup. Once admission closes, dropping one shutdown future leaves cleanup running and other callers can still await completion. The unchanged `()` result establishes task/cleanup completion, not successful electrical stopping: inspect logged driver errors, and do not infer mechanical standstill from software completion. A blocked driver call can delay shutdown indefinitely.

Background tasks own internal state rather than `Arc<Robot>`. Dropping the last public handle requests cancellation and best-effort asynchronous cleanup while the Tokio runtime stays alive, even if another owner retains the motor driver. Dropping one of several handles leaves the controller running. `Drop` performs no synchronous motor I/O and cannot guarantee cleanup after runtime destruction, process termination or power failure. Explicit awaited shutdown is the supported completion path.

## Examples

### Web Server Example

The repository includes a full web server example (`examples/web_server.rs`) with a dual-joystick UI. It uses left M4/right M1 with both motor factors defaulting to +1:

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

### Controller verification on hardware

`controller_check` exercises `Robot`, unlike the raw M1–M4 motor test. It defaults to left M4/right M1, factors +1, and power 0.25. Keep the tracks clear of the surface, use a power that moves your motors safely, and keep a power cutoff accessible. Stop any other process controlling the motor board first.

```bash
cargo run --example controller_check --features pca9685 -- --power 0.25
# Check existing left polarity behavior in a second run:
cargo run --example controller_check --features pca9685 -- --power 0.25 --left-factor -1.0
```

The bounded sequence prints each stage: zero, left-only, zero, right-only, forward, zero, reverse, zero (one second each, including ramp). It then submits a valid left demand with a NaN right demand. Expect an `Expected invalid-input rejection` message and both tracks to remain at zero for another second, followed by explicit shutdown. This does not measure wheel speed or confirm hardware output through sensors. Ctrl-C/SIGTERM during the motion sequence also awaits shutdown.

For the paired-target check, run the same baseline with the optional extension:

```bash
cargo run --example controller_check --features pca9685 -- --power 0.25 --paired-targets
```

After the baseline, expect six additional `paired targets / ...` stages: forward, pivot right, forward, pivot left, forward, zero, each held for one second including ramp. Forward requests `(power, power)`, pivot right `(power, -power)`, and pivot left `(-power, power)` before motor factors. The invalid-input check and explicit shutdown follow as usual; the complete sequence takes about 15 seconds. Check left-only/right-only and pivot directions against your calibrated wiring and the expected behavior, looking for unexpected transient combinations. Independent ramps can produce intermediate output pairs during transitions; barrier-coordinated mock tests verify atomic target acceptance, which visual observation cannot prove.

Check configuration rejection separately:

```bash
cargo run --example controller_check --features pca9685 -- --right-motor 4
cargo run --example controller_check --features pca9685 -- --interval-ms 0
```

Both should print `Controller rejected configuration`, exit with an error and never begin the motion sequence. The example opens and stops the board before creating the controller, so these are not tests of hardware-free startup. Use `--i2c-bus /dev/i2c-1 --i2c-addr 96` to override the connection and `--left-motor`/`--right-motor` to match your wiring. Avoid large factors on hardware: they can saturate at full power; mock tests cover those extremes.

For v0.1.4 stopping verification, repeat the baseline with the corrected left M4/right M1 assignment and the same factors and modest power used for the accepted v0.1.3 run:

```bash
cargo run --example controller_check --features pca9685 -- --power 0.25
```

Observe the `forward → zero` and `reverse → zero` stages: the configured pair should return to the same zero-output behavior as v0.1.3. Expect the invalid-input rejection message and final `Shutdown completed` message, with no driver warnings on healthy hardware. Verify all connected ports remain off after shutdown. The raw full-speed M1–M4 test is optional and appropriate only for independently connected motors whose mechanism permits that sequence.

For v0.1.5 lifecycle verification, use the accepted v0.1.4 configuration with tracks lifted and an accessible cutoff:

```bash
cargo run --example controller_check --features pca9685 -- --power 0.25 --lifecycle-check forward
cargo run --example controller_check --features pca9685 -- --power 0.25 --lifecycle-check reverse
cargo run --example controller_check --features pca9685 -- --power 0.25 --lifecycle-check drop
# Fresh controller: repeat the ordinary baseline after the lifecycle checks.
cargo run --example controller_check --features pca9685 -- --power 0.25
```

Each lifecycle run first performs the baseline, then commands the selected direction for one second including ramp. Forward/reverse modes await shutdown twice and print `Shutdown completed twice`, then keep the process alive for two seconds: outputs should remain off. Drop mode commands forward, drops the only public robot handle, and prints `Last Robot handle dropped`; the runtime and an external motor-driver reference stay alive during the two-second observation window. This tests asynchronous drop cleanup independently of process exit or board destruction. The drop observation window is not a cleanup acknowledgement or a guaranteed stopping deadline. Each run takes roughly 12 seconds under normal conditions; blocked I/O can extend it.

Record version, board/bus/address, wiring, factors, requested power, observed stopping times and pass/fail results. End the test with the power cutoff and investigate unexpected direction, loss of control, driver errors or motion after shutdown. Inject bus faults only in the automated mocks; do not unplug powered motor wiring. These observations supplement the mock race tests and do not establish mechanical standstill.


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
