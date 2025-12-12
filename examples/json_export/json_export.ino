#include <Arduino.h>
#include <ESPCpuMonitor.h>

ESPCpuMonitor cpuMonitor;

#if !ESPCM_HAS_ARDUINOJSON
#error "Install ArduinoJson to run this example."
#endif

#include <ArduinoJson.h>
#include <cmath>

static const uint32_t HISTORY_LOG_PERIOD_MS = 5000;

void setup() {
    Serial.begin(115200);

    CpuMonitorConfig cfg;
    cfg.sampleIntervalMs = 1000; // 1s cadence for telemetry
    cfg.historySize = 12;        // keep a handful of points for charts/debug
    cfg.enableTemperature = true;
    cpuMonitor.init(cfg);

    cpuMonitor.onSample([](const CpuUsageSample &sample) {
        StaticJsonDocument<256> doc;
        cpuMonitor.toJson(sample, doc);
        doc["avgRounded"] = static_cast<int>(roundf(sample.average)); // handy for dashboards

        serializeJson(doc, Serial);
        Serial.println(); // newline-delimited JSON (NDJSON) stream
    });
}

void loop() {
    static uint32_t lastHistoryMs = 0;

    if (cpuMonitor.isReady() && millis() - lastHistoryMs >= HISTORY_LOG_PERIOD_MS) {
        lastHistoryMs = millis();
        const auto hist = cpuMonitor.history();
        if (!hist.empty()) {
            const auto &latest = hist.back();
            Serial.printf("[json_export] history_depth=%u last_avg=%.1f%% last_temp=%.1fC\n",
                          static_cast<unsigned>(hist.size()),
                          latest.average,
                          std::isnan(latest.temperatureC) ? -1.0f : latest.temperatureC);
        }
    }

    delay(50);
}
