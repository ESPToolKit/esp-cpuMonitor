# ESPCpuMonitor

ESPCpuMonitor is a tiny C++17 helper that estimates per-core CPU usage on ESP32 by counting FreeRTOS idle-hook hits. It calibrates a 100% idle baseline, samples on an esp_timer, and keeps history/callbacks ready for charts or alerts.

## CI / Release / License
[![CI](https://github.com/ESPToolKit/esp-cpuMonitor/actions/workflows/ci.yml/badge.svg)](https://github.com/ESPToolKit/esp-cpuMonitor/actions/workflows/ci.yml)
[![Release](https://img.shields.io/github/v/release/ESPToolKit/esp-cpuMonitor?sort=semver)](https://github.com/ESPToolKit/esp-cpuMonitor/releases)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE.md)

## Features
- FreeRTOS idle-hook based sampling with esp_timer; configurable period and calibration window so you can balance responsiveness vs. noise.
- Per-core and averaged CPU usage (%) with timestamps from `esp_timer_get_time`, plus a ring-buffer history (default 60 entries) for charts/debug.
- Thread-safe per-sample callbacks for logging, telemetry, or UI updates; swap `enablePerCore` off to collapse cores to an overall average.
- Manual `sampleNow()` path for users who already have their own schedulers (set `sampleIntervalMs` to `0`).
- Optional CPU temperature readings (current + running average) using the ESP-IDF temperature sensor driver, gracefully disabled when unsupported.
- Optional ArduinoJson export helper to stream samples over HTTP/MQTT/WebSockets alongside the rest of ESPToolKit.

## Examples
- `examples/basic_monitor` – timer-driven sampling with per-core logs and optional temperature readings.
- `examples/manual_sampling` – drives `sampleNow()` manually (no esp_timer), shows how to log history depth and keep calibration/averaging stable during bursty work.
- `examples/json_export` – streams newline-delimited JSON via `toJson()` + ArduinoJson for dashboards/MQTT/serial telemetry.

Create your own `ESPCpuMonitor` instance; the library no longer exposes a built-in global, and only one monitor can be active because idle hooks are shared.

Basic Arduino sketch that prints CPU usage once calibrated:

```cpp
#include <Arduino.h>
#include <ESPCpuMonitor.h>
#include <esp_log.h>
#include <cmath>

ESPCpuMonitor cpuMonitor; // user-owned instance (only one active at a time)

void setup() {
    Serial.begin(115200);

    CpuMonitorConfig cfg;
    cfg.sampleIntervalMs = 1000;   // 1s cadence
    cfg.calibrationSamples = 5;    // treat first 5 periods as "100% idle" baseline
    cfg.historySize = 30;
    cfg.enableTemperature = true;  // disable if your target lacks a temp sensor
    cpuMonitor.init(cfg);

    cpuMonitor.onSample([](const CpuUsageSample &s) {
#if portNUM_PROCESSORS > 1
        ESP_LOGI("CPU", "core0=%.1f%% core1=%.1f%% avg=%.1f%% temp=%.1fC (avg %.1fC)",
                 s.perCore[0], s.perCore[1], s.average,
                 std::isnan(s.temperatureC) ? -1.0f : s.temperatureC,
                 std::isnan(s.temperatureAvgC) ? -1.0f : s.temperatureAvgC);
#else
        ESP_LOGI("CPU", "core0=%.1f%% avg=%.1f%% temp=%.1fC (avg %.1fC)",
                 s.perCore[0], s.average,
                 std::isnan(s.temperatureC) ? -1.0f : s.temperatureC,
                 std::isnan(s.temperatureAvgC) ? -1.0f : s.temperatureAvgC);
#endif
    });
}

void loop() {
    delay(1000);
}
```

When you set `sampleIntervalMs` to `0`, call `sampleNow()` on your monitor instance (for example, `cpuMonitor.sampleNow(sample)`) from your scheduler to drive sampling manually.
If temperature is enabled, `getLastTemperature(current, average)` returns the latest reading and running mean (returns `false` when unsupported or not ready).

## Gotchas
- CPU usage numbers are floats in percent so you can see tiny changes; `1.0` means ~1% busy, not 100%. Feel free to round (`%.0f%%`) in your logs/UI if you prefer whole numbers.
- Allow the calibration window (`calibrationSamples`) to finish before trusting numbers; keep the device as idle as possible during that phase.
- Idle hooks do not run in deep sleep or while interrupts lock out FreeRTOS; readings simply pause in those periods.
- If you disable per-core reporting, each entry in `perCore` mirrors the averaged usage for consistency.
- The esp_timer callback should stay lightweight; heavy work belongs in your own task triggered by `onSample`.
- Some ESP32 variants or Arduino builds omit the internal temperature sensor; in that case `temperatureC`/`temperatureAvgC` stay `NaN` and `getLastTemperature` returns `false`.

## API Reference
- `bool init(const CpuMonitorConfig &cfg = {})` / `void deinit()` – register idle hooks, create the esp_timer (when `sampleIntervalMs > 0`), and reset state.
- `bool isReady() const` – returns true once calibration completed and a sample is stored.
- `bool getLastSample(CpuUsageSample &out) const` / `float getLastAverage() const` – read the latest sample; average returns `-1.0f` until ready.
- `bool getLastTemperature(float &currentC, float &averageC) const` – latest temperature and running average; returns `false` if disabled, unsupported, or not yet sampled.
- `std::vector<CpuUsageSample> history() const` – copy of the ring buffer (size capped by `historySize`, `0` disables storage).
- `bool sampleNow(CpuUsageSample &out)` – immediate sampling, useful when periodic timer is disabled.
- `void onSample(CpuSampleCallback cb)` – subscribe to every stored sample.
- `void toJson(const CpuUsageSample&, JsonDocument &doc)` – ArduinoJson export helper (compiled only when ArduinoJson is available).

`CpuMonitorConfig` knobs:
- `sampleIntervalMs` (default `1000`) – esp_timer period in milliseconds; `0` skips timer creation.
- `calibrationSamples` (default `5`) – number of windows to treat as 100% idle baseline.
- `historySize` (default `60`) – depth of stored samples; set to `0` to disable.
- `enablePerCore` (default `true`) – when `false`, every `perCore` entry is set to the averaged usage.
- `enableTemperature` (default `true`) – enable/disable temperature sensor readings.

`CpuUsageSample` contains `timestampUs`, `perCore[portNUM_PROCESSORS]`, `average` (%), `temperatureC`, and `temperatureAvgC` (NaN when unsupported).

## Restrictions
- ESP32 + FreeRTOS (Arduino-ESP32 or ESP-IDF) with `esp_freertos_hooks.h` available.
- C++17 required; tested with dual-core chips but handles `portNUM_PROCESSORS == 1`.

## Tests
- Build and run `examples/basic_monitor`, `examples/manual_sampling`, and `examples/json_export` via PlatformIO CI or Arduino CLI for a quick smoke test on ESP32 dev boards.

## License
MIT — see [LICENSE.md](LICENSE.md).

## ESPToolKit
- Check out other libraries: <https://github.com/orgs/ESPToolKit/repositories>
- Hang out on Discord: <https://discord.gg/WG8sSqAy>
- Support the project: <https://ko-fi.com/esptoolkit>
- Visit the website: <https://www.esptoolkit.hu/>
