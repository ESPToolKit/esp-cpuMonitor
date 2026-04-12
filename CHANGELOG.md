# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]
### Added
- `examples/manual_sampling` shows manual `sampleNow()` usage without the esp_timer while keeping calibration/history stable.
- `examples/json_export` streams newline-delimited JSON via `toJson()` + ArduinoJson for telemetry pipelines.
- Optional average smoothing in `CpuUsageSample` via `smoothedAverage`, with configurable `smoothingMode` (`None`, `RollingMean`, `Ewma`), rolling window size, and EWMA alpha.
- `getLastSmoothedAverage()` helper for quickly reading the latest trend/baseline value when smoothing is enabled.
- `CpuMonitorConfig::usePSRAMBuffers` toggle to prefer PSRAM-backed internal history and callback container/snapshot storage through `ESPBufferManager`, with automatic fallback to normal heap.
- Token-based measurement API via `startMeasure()` / `stopMeasure(token)` with precise timing fields (`durationUs`, `durationMs`, `durationSec`) and optional per-window CPU usage in `CpuMeasure`.

### Changed
- Removed the library-provided global `cpuMonitor`; create and manage your own `ESPCpuMonitor` instance (only one active monitor at a time) and updated examples/docs accordingly.
- Made `ESPCpuMonitor` non-copyable/movable to prevent accidental double-free of FreeRTOS handles.
- `toJson()` now exports `avgSmoothed` when smoothing is enabled and a smoothed sample is available.
- Baseline calibration now tracks idle rates per microsecond so code-path measurements can estimate CPU usage across arbitrary measurement durations.

### Fixed
- CI now pins PIOArduino Core to `v6.1.19` and installs the ESP32 platform via `pio pkg install`, restoring PlatformIO compatibility with the current `platform-espressif32` package.

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
