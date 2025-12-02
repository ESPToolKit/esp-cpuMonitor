#include <Arduino.h>
#include <ESPCpuMonitor.h>
#include <esp_log.h>
#include <cmath>

void setup() {
    Serial.begin(115200);

    CpuMonitorConfig cfg;
    cfg.sampleIntervalMs = 1000;  // 1s sampling cadence
    cfg.calibrationSamples = 5;   // treat first 5 periods as 100% idle baseline
    cfg.historySize = 20;
    cfg.enableTemperature = true; // toggle temperature readings if your board supports it
    cpuMonitor.init(cfg);

    cpuMonitor.onSample([](const CpuUsageSample &sample) {
#if portNUM_PROCESSORS > 1
        ESP_LOGI("CPU", "core0=%.1f%% core1=%.1f%% avg=%.1f%% temp=%.1fC (avg %.1fC)",
                 sample.perCore[0], sample.perCore[1], sample.average,
                 std::isnan(sample.temperatureC) ? -1.0f : sample.temperatureC,
                 std::isnan(sample.temperatureAvgC) ? -1.0f : sample.temperatureAvgC);
#else
        ESP_LOGI("CPU", "core0=%.1f%% avg=%.1f%% temp=%.1fC (avg %.1fC)",
                 sample.perCore[0], sample.average,
                 std::isnan(sample.temperatureC) ? -1.0f : sample.temperatureC,
                 std::isnan(sample.temperatureAvgC) ? -1.0f : sample.temperatureAvgC);
#endif
    });
}

void loop() {
    // Optional: pull the latest average every few seconds for a quick check
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 3000) {
        lastPrint = millis();
        float avg = cpuMonitor.getLastAverage();
        if (avg >= 0.0f) {
            float temp = 0.0f;
            float tempAvg = 0.0f;
            if (cpuMonitor.getLastTemperature(temp, tempAvg)) {
                Serial.printf("[loop] CPU avg: %.1f%% temp: %.1fC (avg %.1fC)\n", avg, temp, tempAvg);
            } else {
                Serial.printf("[loop] CPU avg: %.1f%% temp: n/a\n", avg);
            }
        } else {
            Serial.println("[loop] calibrating...");
        }
    }
    delay(250);
}
