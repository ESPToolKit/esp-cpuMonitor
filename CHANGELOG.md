# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]
- No changes yet.

## [1.0.1] - 2025-12-03
### Fixed
- Idle hook registration now handles the newer IDF/Arduino APIs that return `esp_err_t`, preventing false failures on startup.

### Changed
- Default to the modern `driver/temperature_sensor.h` when available, with an opt-in legacy path to avoid deprecated driver warnings.
- README clarifies that CPU usage is reported as floating-point percentages so sub-1% load is visible.
- CI updated for ESP32-P4: correct board id (`esp32-p4-evboard`) and exclusion of unsupported P4 module configs.

## [1.0.0] - 2025-12-02
### Added
- Initial ESPCpuMonitor release: FreeRTOS idle-hook based CPU usage sampling with calibration, periodic timer or manual `sampleNow`, per-core and average reporting, ring-buffered history, and per-sample callbacks.
- Optional CPU temperature sampling (current and running average) exposed in `CpuUsageSample`, ArduinoJson export helper, and `getLastTemperature`.
- Basic Arduino example sketch.
