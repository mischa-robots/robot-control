# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

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
