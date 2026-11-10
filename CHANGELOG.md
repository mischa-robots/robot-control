# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

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
