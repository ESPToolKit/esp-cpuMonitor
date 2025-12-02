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
- Optional ArduinoJson export helper to stream samples over HTTP/MQTT/WebSockets alongside the rest of ESPToolKit.

## Examples
Basic Arduino sketch that prints CPU usage once calibrated:

```cpp
#include <Arduino.h>
#include <ESPCpuMonitor.h>
#include <esp_log.h>

void setup() {
    Serial.begin(115200);

    CpuMonitorConfig cfg;
    cfg.sampleIntervalMs = 1000;   // 1s cadence
    cfg.calibrationSamples = 5;    // treat first 5 periods as "100% idle" baseline
    cfg.historySize = 30;
    cpuMonitor.init(cfg);

    cpuMonitor.onSample([](const CpuUsageSample &s) {
#if portNUM_PROCESSORS > 1
        ESP_LOGI("CPU", "core0=%.1f%% core1=%.1f%% avg=%.1f%%",
                 s.perCore[0], s.perCore[1], s.average);
#else
        ESP_LOGI("CPU", "core0=%.1f%% avg=%.1f%%", s.perCore[0], s.average);
#endif
    });
}

void loop() {
    delay(1000);
}
```

When you set `sampleIntervalMs` to `0`, call `cpuMonitor.sampleNow(sample)` from your scheduler to drive sampling manually (useful in tight control loops).

## Gotchas
- Allow the calibration window (`calibrationSamples`) to finish before trusting numbers; keep the device as idle as possible during that phase.
- Idle hooks do not run in deep sleep or while interrupts lock out FreeRTOS; readings simply pause in those periods.
- If you disable per-core reporting, each entry in `perCore` mirrors the averaged usage for consistency.
- The esp_timer callback should stay lightweight; heavy work belongs in your own task triggered by `onSample`.

## API Reference
- `bool init(const CpuMonitorConfig &cfg = {})` / `void deinit()` – register idle hooks, create the esp_timer (when `sampleIntervalMs > 0`), and reset state.
- `bool isReady() const` – returns true once calibration completed and a sample is stored.
- `bool getLastSample(CpuUsageSample &out) const` / `float getLastAverage() const` – read the latest sample; average returns `-1.0f` until ready.
- `std::vector<CpuUsageSample> history() const` – copy of the ring buffer (size capped by `historySize`, `0` disables storage).
- `bool sampleNow(CpuUsageSample &out)` – immediate sampling, useful when periodic timer is disabled.
- `void onSample(CpuSampleCallback cb)` – subscribe to every stored sample.
- `void toJson(const CpuUsageSample&, JsonDocument &doc)` – ArduinoJson export helper (compiled only when ArduinoJson is available).

`CpuMonitorConfig` knobs:
- `sampleIntervalMs` (default `1000`) – esp_timer period in milliseconds; `0` skips timer creation.
- `calibrationSamples` (default `5`) – number of windows to treat as 100% idle baseline.
- `historySize` (default `60`) – depth of stored samples; set to `0` to disable.
- `enablePerCore` (default `true`) – when `false`, every `perCore` entry is set to the averaged usage.

`CpuUsageSample` contains `timestampUs`, `perCore[portNUM_PROCESSORS]`, and `average` (%).

## Restrictions
- ESP32 + FreeRTOS (Arduino-ESP32 or ESP-IDF) with `esp_freertos_hooks.h` available.
- C++17 required; tested with dual-core chips but handles `portNUM_PROCESSORS == 1`.

## Tests
- Build and run `examples/basic_monitor` via PlatformIO CI or Arduino CLI for a quick smoke test on ESP32 dev boards.

## License
MIT — see [LICENSE.md](LICENSE.md).

## ESPToolKit
- Check out other libraries: <https://github.com/orgs/ESPToolKit/repositories>
- Hang out on Discord: <https://discord.gg/WG8sSqAy>
- Support the project: <https://ko-fi.com/esptoolkit>
