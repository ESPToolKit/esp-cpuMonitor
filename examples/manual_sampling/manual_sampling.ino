#include <Arduino.h>
#include <ESPCpuMonitor.h>
#include <esp_log.h>

static const uint32_t SAMPLE_PERIOD_MS = 500;
ESPCpuMonitor cpuMonitor;

// Simple spin to create a small CPU bump for the manual sampler.
static void spinWork(uint32_t durationMicros) {
    const uint32_t start = micros();
    while ((uint32_t)(micros() - start) < durationMicros) {
        asm volatile("nop");
    }
}

void setup() {
    Serial.begin(115200);

    CpuMonitorConfig cfg;
    cfg.sampleIntervalMs = 0;      // manual sampling only
    cfg.calibrationSamples = 8;    // give the idle baseline a few readings
    cfg.historySize = 10;
    cfg.enablePerCore = false;     // focus on averaged load
    cfg.enableTemperature = false; // keep it portable on targets without temp sensor
    cpuMonitor.init(cfg);

    cpuMonitor.onSample([](const CpuUsageSample &sample) {
        ESP_LOGI("CPU", "avg=%.1f%% ts=%llu",
                 sample.average,
                 static_cast<unsigned long long>(sample.timestampUs));
    });
}

void loop() {
    static uint32_t lastSampleMs = 0;

    // Simulate bursty work after calibration: spin for ~5ms while in a "busy" window.
    const bool busyWindow = cpuMonitor.isReady() && (millis() / 3000) % 2 == 0;
    if (busyWindow) {
        spinWork(5000);
    }

    if (millis() - lastSampleMs >= SAMPLE_PERIOD_MS) {
        lastSampleMs = millis();

        CpuUsageSample sample{};
        bool ready = cpuMonitor.sampleNow(sample); // drive sampling yourself
        if (!ready) {
            Serial.println("[manual_sampling] calibrating baseline...");
        } else {
            Serial.printf("[manual_sampling] avg=%.1f%% history_depth=%u\n",
                          sample.average,
                          static_cast<unsigned>(cpuMonitor.history().size()));
        }
    }

    delay(10);
}
