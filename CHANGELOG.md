# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

## [0.1.0] - 2025-12-02
### Added
- Initial ESPCpuMonitor library with FreeRTOS idle-hook based CPU usage sampling for ESP32.
- Baseline calibration over configurable periods plus periodic timer sampling or manual `sampleNow`.
- Per-core and average CPU load reporting, ring-buffered history, and per-sample callbacks.
- Optional ArduinoJson export helper and a basic Arduino example sketch.
