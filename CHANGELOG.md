# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

## 0.1.9

### Fixed

Web server example:

- Serialize WebSocket and HTTP motion through one worker with a 32-command waiting queue. Admit WebSocket commands in receive order instead of spawning independent motion tasks.
- Claim control on the first valid, admitted WebSocket command. Other sessions and HTTP cannot drive or stop while that owner holds control; a non-owner disconnect has no motor effect.
- On owner disconnect, discard pending commands and request the existing smooth stop before admitting another owner. A command already in flight completes before that stop; old queued demands cannot restart motion afterward.
- Give zero-demand stops a reserved slot even when the ordinary queue is full. Stops supersede older queued commands; subsequent accepted commands run after the stop. Invalid WebSocket messages close the session and release any ownership.

### Changed

- Web command responses are JSON: `accepted` confirms Robot target acceptance, and errors include `code` and `error`. HTTP reports invalid input as 400, ownership/cancelled commands as 409, queue overflow as 429 and controller fault/shutdown as 503. The left/right request shape and finite clamping remain unchanged.
- Browser status shows acceptance, contention and errors, clears local demand on rejection/disconnection, and never retries stored commands. Queue overload or controller errors close the browser's stream to release control.
- Log server rejections, malformed responses and WebSocket connection errors to the browser console for debugging; normal status/acceptance and routine stop supersessions remain quiet.

### Added

- Mock tests for ordered commands under delay, bounded overflow, priority stops, ownership, disconnect/reconnect, invalid requests, HTTP conflicts and fault diagnostics; a Robot integration check retains smooth-stop behavior.
- Browser connection tests and an operator procedure for two-tab ownership and HTTP conflicts.

Library API, electrical behavior, defaults and left M4/right M1 wiring are unchanged. Ownership applies within one web server process. Silent connection loss and finite command expiry remain v0.1.10 work; commands still persist until replaced/stopped. Hardware acceptance is pending operator testing.

## 0.1.8

### Fixed

Webserver example:
- Retain the robot at the web server's top level and await cleanup on normal completion, bind/startup errors and serve errors. Preserve the original server error while reporting controller/cleanup failures.
- Handle Ctrl-C and SIGTERM explicitly: close motor command admission and await robot shutdown before stopping HTTP workers and joining the server. Long-lived WebSockets do not delay the initial robot shutdown.
- Report board/configuration startup errors instead of panicking; attempt an exhaustive board stop if robot configuration is rejected after the board opens.

### Changed

Webserver example:
- `/health` now returns JSON controller state: HTTP 200 for `Running`, HTTP 503 for `ShuttingDown`, `Shutdown` or `Faulted`, with output certainty and fault/cleanup diagnostics. This replaces the unconditional `Healthy` text response; it is not a hardware self-test.
- Enable web example lifecycle/cleanup logs in the default logging filter.

### Added

- Mock web lifecycle and HTTP health tests for startup/serve errors, awaited cleanup, injected shutdown signals, error precedence and background driver faults.
- Operator verification for Ctrl-C, SIGTERM and an occupied HTTP port without a competing motor controller.

Library API, motor behavior and web command lifetime/ownership are unchanged. SIGKILL, power loss and indefinitely blocked hardware I/O remain outside the cleanup guarantee.

## 0.1.7

### Fixed

- Advance motor ramps using the configured nominal rate (`adjustment_step / adjustment_interval`) and monotonic elapsed time. Cap elapsed time at one interval so a delayed update never exceeds the existing maximum step.
- Schedule against the original deadline grid, including driver-lock/I/O delays, and skip missed ticks instead of replaying them or adding a fresh full sleep after each output batch.

### Added

- Bounded `RobotStatus::timing` diagnostics (`RobotTiming`): saturating overrun/skipped-tick counters and last/maximum uncapped update spacing, retained after shutdown.
- Pure and controlled-time tests covering early/jittered updates, long gaps, stop/reversal, target changes, rate equivalence, bounds, skipped ticks, driver-lock delays and distant-deadline cancellation.
- `controller_check --adjustment-step` for slower observable ramps and optional `--trace-output` for timestamped output-write results, with a hardware timing procedure.
- Example-only bounds for `controller_check`: adjustment step 0.01–0.1 and interval 10–20 ms, checked before opening hardware.

Existing config fields, call signatures, 0.1/10 ms defaults, reversal policy, electrical values and left M4/right M1 example wiring are preserved. Delays and the elapsed-time cap can extend ramp duration; no large catch-up step or timing-fault policy is introduced.

## 0.1.6

### Fixed

- Latch the first background motor-write failure, clear both pending targets, close command admission and run the existing joined, exhaustive cleanup. A failed left write skips the right drive write; neither motor receives later adjustment writes after a fault.
- Preserve background and final-stop diagnostics across cleanup and repeated shutdown. Faulted `drive`/`stop` calls return the existing `RobotError::Motor(MotorError::Pwm(...))` with diagnostic context; recovery requires a fresh robot after cleanup.

### Added

- `Robot::status()` and the additive `RobotStatus`, `RobotPhase` and `RobotOutputState` types. Snapshots expose lifecycle, cleanup completion, the first background failure and final cleanup failure, with each diagnostic bounded to 4096 UTF-8 bytes. Output remains `Unknown` until a final stop succeeds; `Stopped` means driver-reported success, not measured standstill.
- Mock regression coverage for left/right and zero-output failures, queued commands, shutdown/fault races, cleanup failure, adjustment-task panic, persistent faults and bounded diagnostics.
- Hardware-check status output and a hardware-free `fault_check` example, including an optional cleanup failure.

Existing public signatures, config literals and error variants remain compatible. Ramping, electrical behavior and left M4/right M1 example wiring are preserved.

## 0.1.5

### Fixed

- Close command admission, cancel and join the adjustment task, then perform the final exhaustive stop. No adjustment writes occur after awaited `shutdown` returns.
- Share one cleanup task across repeated/concurrent shutdown calls. Cancelling an awaiter leaves cleanup running and retains the join handle for other callers.
- Remove the background task's ownership of `Arc<Robot>`. Dropping the last public handle requests cancellation and best-effort asynchronous cleanup even when an external driver reference survives.
- Correct shutdown/drop documentation: explicit awaited shutdown establishes task and cleanup completion; stop failures are logged through the existing `()` API. Drop requires a live runtime and does not synchronously finish hardware I/O.

### Added

- Mock lifecycle tests with blocked driver writes/stops covering ordered cleanup, command rejection, concurrent/repeated shutdown, cancelled awaiters, last-handle drop, surviving driver references and stop failures.
- `controller_check --lifecycle-check forward|reverse|drop` and operator instructions for bounded motion followed by shutdown/drop and a live-runtime observation window.

Public signatures, error variants, ramping and electrical behavior remain unchanged. Examples retain left M4/right M1 and +1 factors.

## 0.1.4

### Fixed

- The default `MotorDriver::stop_all` attempts M1–M4 in order even when earlier stops fail, returning all failed motor diagnostics through the existing `MotorError::Pwm` variant.
- `Pca9685MotorBoard::stop_motor` attempts all six writes in the existing zero-output sequence even after a write fails, reporting every failed motor/channel/operation and its underlying diagnostic.
- Correct swapped left/right assignments in the web server and controller hardware-check examples: left is M4, right is M1. Update the matching mock output expectations and hardware instructions.

### Added

- Hardware-free failure-injection tests covering every subset of motor-stop failures and every subset of individual stop-write failures on all four channel maps, including successful write order and zero values.
- Documentation of custom `stop_all` override responsibilities, uncertain output after stop errors, and the v0.1.4 operator verification procedure.

Public signatures and error variants are unchanged. Healthy-board write order, electrical values, polarity, prescale and ramping are preserved.

## 0.1.3

### Fixed

- Accept left/right targets atomically in `drive` and `stop`, and read one paired snapshot per adjustment tick so concurrent commands cannot produce mixed targets.
- Synchronize command admission and shutdown cancellation with the same target lock; commands waiting behind shutdown cannot replace targets after admission closes.

### Added

- Barrier-coordinated concurrent drive/stop regression tests, deterministic queued-command admission checks, and mock adjustment-loop checks for complete pairs.
- Optional `--paired-targets` sequence in `controller_check` for bounded alternating forward/pivot hardware checks, with a README procedure.

Public APIs, defaults, per-motor ramping and electrical behavior are unchanged. Target acceptance is atomic; physical motor writes remain sequential.

## 0.1.2

### Fixed

- Validate robot configuration before starting the controller task: motor ports must differ, adjustment interval and step must be positive, step and factors must be finite, and the derived step/interval rate must be positive and finite in `f32`.
- Reject non-finite drive inputs without replacing either motor target.
- Clamp factored motor demands to the supported range, retaining finite gain factors (including zero and negative factors) and preventing out-of-range PWM output.

### Added

- Mock regression tests for invalid configuration/commands, extreme finite factors, PWM bounds, and compatibility with existing config literals, custom drivers and error matches.
- Bounded `controller_check` hardware example and README procedure for validating the changes on M1/M4, including negative polarity and rejected commands/configurations.

Existing public signatures, motor defaults, ramp interval, PWM mapping and shutdown behavior are unchanged.

## 0.1.1 - 2026-11-10

### Added

- Motor test example (`examples/test.rs`) to exercise M1–M4 sequentially in both directions, with configurable I2C bus and address.
- README instructions for running the motor test example.
- CHANGELOG.md

### Changed

- Web server example now uses M1 for the left motor and M4 for the right motor (previously M2).
- Web server's left motor factor now defaults to `1.0` (previously `-1.0`), matching the right motor factor.

### Fixed

- Web server CLI accepts negative values for `--left-factor` and `--right-factor` without requiring `=`.
- Joystick UI connects to the WebSocket using the page's host, port, and HTTP/HTTPS scheme instead of hard-coding port 8000, and only sends drive commands while connected.

## 0.1.0 - 2026-04-17

### Added

- Initial release with a hardware-abstracted motor driver, smooth motor ramping, optional PCA9685 support, and a web server example with joystick controls.
