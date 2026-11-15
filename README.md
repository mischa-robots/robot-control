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

`drive(left, right)` accepts both targets atomically, and `stop()` atomically replaces the pair with zero. Each adjustment tick reads one complete pair, even when commands run concurrently. Commands waiting behind shutdown recheck admission under the same lock and return `RobotError::Shutdown` without changing targets once cancellation has begun, or the retained diagnostic if the controller faulted.

Successful `drive` means target acceptance. Each motor still ramps independently, and the driver writes the left and right outputs sequentially. Paired acceptance does not change ramp timing, polarity, PWM mapping or electrical stopping.

### Exhaustive stopping (0.1.4)

The default `MotorDriver::stop_all()` attempts M1–M4 in order even if an earlier stop fails. `Pca9685MotorBoard::stop_motor()` similarly attempts all six writes: reference-channel on/off, forward-channel on/off, then backward-channel on/off, all with value zero. The healthy-board write sequence and electrical behavior are unchanged.

Failures are collected into the existing `MotorError::Pwm` variant. Messages identify every failed motor and retain its diagnostic; PCA9685 write failures also identify the channel and operation. A returned error means actual output is uncertain even though the remaining stops were attempted. Custom drivers overriding `stop_all()` must implement their own exhaustive stopping behavior.

`Robot::stop()` still accepts a smooth ramp to zero; the call does not confirm subsequent hardware application. `Robot::shutdown()` logs stop failures and retains its existing `()` return type. As of 0.1.6, failures are also visible through `Robot::status()`.

### Ordered shutdown and task ownership (0.1.5)

`robot.shutdown().await` closes command admission under the paired target lock, requests cancellation, waits for the adjustment task to terminate, and then finishes the final `stop_all()` attempt. New `drive` and `stop` calls return `RobotError::Shutdown` once admission closes normally; latched faults take precedence and return their diagnostic. No adjustment writes occur after shutdown returns. In-flight driver I/O must finish first; shutdown does not abort it or wait for a smooth ramp to zero.

Repeated and concurrent shutdown calls wait for the same cleanup. Once admission closes, dropping one shutdown future leaves cleanup running and other callers can still await completion. The unchanged `()` result establishes task/cleanup completion, not successful electrical stopping: inspect logged driver errors, and do not infer mechanical standstill from software completion. A blocked driver call can delay shutdown indefinitely.

Background tasks own internal state rather than `Arc<Robot>`. Dropping the last public handle requests cancellation and best-effort asynchronous cleanup while the Tokio runtime stays alive, even if another owner retains the motor driver. Dropping one of several handles leaves the controller running. `Drop` performs no synchronous motor I/O and cannot guarantee cleanup after runtime destruction, process termination or power failure. Explicit awaited shutdown is the supported completion path.

### Latched faults and status (0.1.6)

A background output failure now closes command admission, clears both pending targets and ends adjustment before the existing exhaustive cleanup runs. This includes failures writing a zero output. If the left write fails, the right drive write is skipped. A previously accepted or waiting command cannot restart motion after the fault latches. Subsequent `drive` and `stop` calls return `RobotError::Motor(MotorError::Pwm(...))` containing the retained diagnostic. After cleanup, investigate the fault and create a fresh `Robot` to resume; repeated shutdown does not reset a fault or retry cleanup.

`robot.status().await` returns a snapshot without hardware I/O:

| Field | Meaning |
| --- | --- |
| `phase` | `Running`, `ShuttingDown`, `Shutdown` or latched `Faulted` |
| `output` | `Unknown` until the final `stop_all()` succeeds; `Stopped` afterward |
| `fault` | First background output/task failure, retained after shutdown |
| `cleanup_error` | Final stop failure, stored separately from the original fault |
| `cleanup_complete` | Cleanup finished, even when its stop attempt failed |
| `timing` | Bounded elapsed-time and skipped-tick diagnostics, added in 0.1.7 |

The two optional diagnostics each retain at most 4096 UTF-8 bytes, including a truncation marker if needed. Rejected commands do not grow an error history. The new types are exported as `RobotStatus`, `RobotPhase` and `RobotOutputState`; existing error enums and config fields are unchanged.

`Running` means commands are accepted, not that hardware passed a self-test. `drive().await` confirms target acceptance only. While running, output status remains conservatively `Unknown`, even at a zero target. `Stopped` means the final driver stop returned success; it is not sensor feedback or proof of mechanical standstill and cannot account for writes through external driver references. Successful fault cleanup therefore reports `Faulted` with `Stopped`; failed cleanup reports `Faulted` with `Unknown`. Use awaited shutdown to join cleanup, then inspect status.

### Elapsed-time ramping (0.1.7)

The nominal power slew rate is `adjustment_step / adjustment_interval` in seconds. Defaults remain 0.1 per 10 ms, or 10 power units/s. Each update moves toward its target by rate times monotonic elapsed time, bounded by the remaining distance. The first update still waits one interval. Actual elapsed time is sampled after waiting for the driver and control locks and includes the previous output batch's I/O time.

Elapsed time is capped at one configured interval: no update changes power by more than `adjustment_step`, even after a long delay. A shorter gap produces a proportionally smaller change. Deadlines stay on the original schedule and missed ticks are skipped, including those passed during I/O; the controller does not replay overdue ticks or add I/O duration to a full sleep each iteration. Jitter and sustained overload can make ramps take longer than their nominal duration. Reversal still follows the existing ramp through zero without an added dwell policy.

`robot.status().await.timing` returns `RobotTiming` with these diagnostics:

| Field | Meaning |
| --- | --- |
| `overruns` | Updates with elapsed time greater than the configured interval, including ordinary timer jitter |
| `skipped_ticks` | Missed scheduled ticks skipped after a delayed update or output batch |
| `last_elapsed` | Actual spacing before the last update, before applying the elapsed-time cap |
| `max_elapsed` | Largest observed update spacing, also uncapped |

Counters saturate at `u64::MAX`; there is no growing history or per-overrun warning stream. Durations start at zero and diagnostics remain available after shutdown or a fault. Overruns alone do not latch a fault in this release. These are controller timing observations, not wheel-speed measurements or a real-time stopping guarantee.

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
| `/ws` | WebSocket | Drive commands (`{"left": 0.5, "right": 0.5}`); one controlling session |
| `/drive?left=0.5&right=0.5` | GET | Drive command when no WebSocket owner holds control |
| `/health` | GET | JSON controller state; 200 running, 503 shutting down/shutdown/faulted |

Since v0.1.8, `/health` replaces the unconditional `Healthy` text response with JSON. A normal running response is:

```json
{"phase":"Running","output":"Unknown","fault":null,"cleanup_error":null,"cleanup_complete":false}
```

This reports lifecycle and output certainty, not a hardware self-test or sensed motion. A fault remains unhealthy even if cleanup successfully stopped the outputs. During shutdown, the endpoint may briefly report 503 before the server closes its connections; it is unavailable once the server exits.

The web example retains the robot until server completion and awaits cleanup on bind/serve failure, Ctrl-C and SIGTERM. On a shutdown signal it closes motion admission and finishes robot shutdown before stopping HTTP workers, so open WebSockets do not postpone motor cleanup. The default log filter includes the final `robot cleanup completed` status and fault/cleanup warnings. The original server error is preserved if cleanup also fails; a cleanup/controller failure otherwise produces an error exit. SIGKILL, power loss and indefinitely blocked I/O cannot be handled by this shutdown path. Finite command expiry remains a later checkpoint.

#### Ordered web control and ownership (v0.1.9)

Opening a WebSocket does not claim control. Its first valid, admitted left/right command claims ownership until disconnect, including when the owner commands zero. Additional sessions can connect, but their drive/stop requests receive `busy`; closing a non-owner session does not stop the robot. HTTP commands, including zero demand, are also rejected while a WebSocket owns control. This coordinates clients of one server, not separate processes opening the board.

One worker applies admitted commands in order through `Robot`. At most 32 ordinary commands wait, plus one in-flight command and one reserved pending stop. Queue overflow rejects the new command without claiming ownership. A zero/zero command uses the reserved stop slot and cancels older queued commands with `superseded`; repeated pending stops coalesce. Later admitted commands run after the stop. These stops use ordinary ramp-to-zero behavior.

Owner disconnect cancels all pending commands with `disconnected` and requests a priority smooth stop after any in-flight command. New ownership and HTTP commands are blocked until that stop target is accepted by `Robot`, which does not mean the ramp or mechanical stopping has finished. A new session must submit a fresh command; queued commands from the old session cannot restart motion. Malformed, non-finite or unsupported WebSocket messages close the offending session and release any ownership. Invalid HTTP requests do not acquire ownership or affect an existing owner.

WebSocket connections initially receive `{"status":"ready"}`. Successful command acknowledgements and HTTP responses use `{"status":"accepted"}`; this confirms target acceptance, not hardware application. Errors use JSON such as `{"code":"busy","error":"..."}`. The example's HTTP success body therefore changes from `Driving` to JSON in this release:

| HTTP status | Error code | Meaning |
| --- | --- | --- |
| 400 | `invalid` | Missing, malformed or non-finite input |
| 409 | `busy` | Another session owns control, or owner release is pending |
| 409 | `superseded`, `disconnected` | A previously queued command was cancelled |
| 429 | `queue_full` | Waiting command capacity exhausted |
| 503 | `fault`, `shutdown` | Controller failure or closed command admission |

The browser displays ownership/errors and clears local demands on rejection or connection loss. A busy response does not retry the rejected command; another deliberate joystick action can try again after the owner releases control. On overload or controller errors, the browser closes its stream to release ownership; reload and give fresh input to reconnect. No automatic command replay or heartbeat is added. Commands, including HTTP commands, remain persistent, and a silently broken connection may retain ownership and motion until the server detects its closure. Finite expiry belongs to v0.1.10.

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
│  │  ├─ Shared command queue + single owner    │ │
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

For v0.1.6, repeat the baseline with the accepted v0.1.5 wiring, factors and modest power:

```bash
cargo run --example controller_check --features pca9685 -- --power 0.25
```

The example prints status at startup, after each baseline stage and after explicit shutdown. On healthy hardware, expect `Running` with `Unknown` during the sequence, then `Shutdown`, `Stopped`, `cleanup_complete: true`, and no fault/cleanup diagnostic after shutdown. Motion and stopping should match v0.1.5. A successful accepted command is not reported as verified hardware motion.

Demonstrate fault reporting on the robot host without accessing hardware:

```bash
cargo run --example fault_check --no-default-features
cargo run --example fault_check --no-default-features -- --cleanup-failure
```

Both runs accept one target, inject a mock left M4 output failure, print stop attempts for M1–M4 and reject a later drive command. The first ends with `Faulted`/`Stopped`; the second retains both output and cleanup diagnostics and ends with `Faulted`/`Unknown`. Both report cleanup complete and keep the fault after repeated shutdown. The mock run normally finishes in under a second and has a two-second failure-notification guard. No bus disconnection or live wiring fault is needed.

For v0.1.7 timing verification, first repeat the ordinary baseline with the accepted v0.1.6 configuration. Then use a slower nominal rate of 1 power unit/s, keeping tracks lifted and the power cutoff accessible:

The example accepts `--adjustment-step` from 0.01 through 0.1 and `--interval-ms` from 10 through 20, inclusive. Values outside these ranges are rejected before opening the hardware. These simple example limits retain the defaults and the timing checks below; they are not motor stall or thermal protection. The library's configuration range is unchanged.

```bash
cargo run --example controller_check --features pca9685 -- --power 0.5 --adjustment-step 0.01 --interval-ms 10 --trace-output
# Optional comparison: the same nominal rate with a 20 ms interval.
cargo run --example controller_check --features pca9685 -- --power 0.5 --adjustment-step 0.02 --interval-ms 20 --trace-output
# Restore the default 0.1 / 10 ms settings by omitting the timing options.
cargo run --example controller_check --features pca9685 -- --power 0.5
```

Choose a modest power that moves your motors; 0.5 is an example. With +1 factors, the first `left only` request starts M4 from zero and should reach approximately PWM 2048 (0.5 power) in roughly 0.5 seconds under light load. Use the stage's `t=...s` request timestamp and the first corresponding `output ... motor=Motor4 pwm=2048 ... ok=true` line to estimate the time. At power 0.6 the nominal ramp is 0.6 seconds and the target PWM is 2457. Allow for tick phase, rounding, jitter and I/O time; compare requested-output timestamps, not wheel RPM.

Output trace timestamps are taken after each driver call returns and share the stage clock; `ok=true` reports driver success, not sensed motion. Trace printing itself adds load, so also inspect the status timing counters and compare a run without tracing. Repeat under representative host/web activity without running a second controller against the same board. Record requested rate, observed ramp duration, timing diagnostics and differences from v0.1.6. Large delays must not create large power jumps. The example still uses the bounded baseline and explicit shutdown; the raw M1–M4 test's independent timing is unchanged.

#### Web lifecycle verification (v0.1.8)

Repeat the controller baseline first, with the same wiring/factors as the accepted v0.1.7 configuration. For these web checks, keep tracks lifted and use a modest power already shown to move your motors. Run only one motor controller at a time. The commands below use default factors and localhost; apply your tested factors/bus/address when starting the server.

Build once and run the binary directly so signals target the server rather than a Cargo wrapper:

```bash
cargo build --example web_server --features pca9685
RUST_LOG=robot_control=info,web_server=info,actix_web=info \
  target/debug/examples/web_server --host 127.0.0.1 --port 8000
```

In a second terminal, check health and request modest motion (0.25 is only an example):

```bash
curl -i http://127.0.0.1:8000/health
curl --fail 'http://127.0.0.1:8000/drive?left=0.25&right=0.25'
```

Expect HTTP 200 with `phase: Running`, `output: Unknown`, and null fault/cleanup diagnostics. Press Ctrl-C in the server terminal while motion is commanded. Expect `robot cleanup completed` with `Shutdown`, `Stopped`, `cleanup_complete: true`, and no fault/cleanup error, before the process exits. Confirm outputs remain off after exit. Repeat with a browser WebSocket open if desired; the server should not wait for the browser to disconnect before stopping the robot. These web commands remain persistent in v0.1.8, so terminate the server promptly after checking motion.

After the first server has exited, repeat using SIGTERM. In one terminal:

```bash
RUST_LOG=robot_control=info,web_server=info,actix_web=info \
  target/debug/examples/web_server --host 127.0.0.1 --port 8000 &
web_pid=$!
```

Once startup completes, run the health/drive requests from the other terminal. Then return to the launch terminal to signal the server directly and wait for cleanup and exit:

```bash
kill -TERM "$web_pid"
wait "$web_pid"
```

Execute the signal commands only after requesting motion. Expect the same final cleanup status and a successful exit on healthy hardware. Do not use SIGKILL for this check.

For bind failure, start with the robot stopped and all motor-controller processes exited. Occupy port 8000 using a server that never accesses the motor board:

```bash
python3 -m http.server 8000 --bind 127.0.0.1
```

Leave that process running and, in a second terminal, start the web example on the same address/port using the foreground command above. Expect a nonzero exit with the original address-in-use error, preceded by completed robot cleanup with `Shutdown`/`Stopped` and no cleanup errors. No drive command is needed; verify the motors stay off. Stop the Python server with Ctrl-C afterward. Do not use a second robot web server to occupy the port. Serve failures and background driver faults are covered by mocks; do not induce live wiring faults.

#### Web ownership verification (v0.1.9)

Repeat the controller baseline and v0.1.8 lifecycle checks first. Keep tracks lifted, use your previously accepted modest power and factors, and keep the power cutoff accessible. Start one web server, then open two tabs (A and B) at its UI. Reload both tabs to load the matching v0.1.9 browser files.

1. Move a joystick in A. It should report that you control the robot. Releasing the joystick commands smooth zero but retains ownership.
2. Try the joysticks in B: it should report that another client controls the robot, and must not change the outputs. Closing B must not stop A's motion. To hold a modest demand while switching tabs, you can use A's browser console: `sendDriveCommand(0.25, 0.25)`, followed promptly by `sendDriveCommand(0, 0)` to stop. Use your accepted power; commands do not expire yet.
3. While A owns control, run the HTTP requests below on the robot host. Both must return HTTP 409 with `code: busy` and leave A's target unchanged, including the zero request.

```bash
curl -i 'http://127.0.0.1:8000/drive?left=0.25&right=0.25'
curl -i 'http://127.0.0.1:8000/drive?left=0&right=0'
```

4. With a modest command active in A, close A. Expect the ordinary smooth stop. Reopen a tab: connection alone must not start motion. Deliberate new input can claim control after the previous stop request is accepted.
5. Close all controlling tabs. Repeat the two HTTP requests in order, promptly issuing zero after the motion check. Expect HTTP 200 and `status: accepted` for both. Also try a missing or invalid parameter: expect HTTP 400 JSON without changing demand.
6. Finish with Ctrl-C or SIGTERM and confirm the v0.1.8 completed-cleanup status and outputs remaining off.

Do not use network disconnection to test expiry in this version, and do not flood the real robot to test overflow. Queue saturation, stale-command rejection and artificial delays are covered by mocks. Record which tab owned control, observed busy responses and stop behavior.

Record version, board/bus/address, wiring, factors, requested power, observed stopping times and pass/fail results. End the test with the power cutoff and investigate unexpected direction, loss of control, driver errors or motion after shutdown. Inject bus faults only in the automated mocks; do not unplug powered motor wiring. These observations supplement the mock race tests and do not establish mechanical standstill.


## Feature Flags

| Flag | Default | Description |
|------|---------|-------------|
| `pca9685` | off | Enables `Pca9685MotorBoard` with real I2C hardware support via `linux-embedded-hal` and `pwm-pca9685` |

## Development

```bash
# Run tests (no hardware needed)
cargo test
cargo test --example web_server --all-features

# Browser connection tests (Node.js with the built-in test runner)
node --test tests/web_control.test.cjs

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
