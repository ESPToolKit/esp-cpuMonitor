#pragma once

#if defined(ARDUINO)
#include <Arduino.h>
#elif defined(__has_include)
#if __has_include(<Arduino.h>)
#include <Arduino.h>
#endif
#endif

#if defined(__has_include)
#if __has_include(<ArduinoJson.h>)
#include <ArduinoJson.h>
#define ESPCM_HAS_ARDUINOJSON 1
#endif
#endif

#ifndef ESPCM_HAS_ARDUINOJSON
#define ESPCM_HAS_ARDUINOJSON 0
#endif

#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <vector>

extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
}

#include <esp_timer.h>
#include <esp_freertos_hooks.h>

#if defined(__has_include)
#if defined(ARDUINO)
#if __has_include(<driver/temp_sensor.h>)
#define ESPCM_HAS_TEMP_SENSOR_OLD 1
#include <driver/temp_sensor.h>
#endif  // Arduino prefers legacy temp_sensor driver; skip new driver to avoid missing symbols
#else
#if __has_include(<driver/temperature_sensor.h>)
#define ESPCM_HAS_TEMP_SENSOR_NEW 1
#include <driver/temperature_sensor.h>
#elif __has_include(<driver/temp_sensor.h>)
#define ESPCM_HAS_TEMP_SENSOR_OLD 1
#include <driver/temp_sensor.h>
#endif
#endif
#endif

#if defined(__has_include)
#if __has_include(<soc/soc_caps.h>)
#include <soc/soc_caps.h>
#endif
#endif

#ifndef SOC_TEMP_SENSOR_SUPPORTED
#define SOC_TEMP_SENSOR_SUPPORTED 0
#endif

#if (SOC_TEMP_SENSOR_SUPPORTED) && (ESPCM_HAS_TEMP_SENSOR_NEW || ESPCM_HAS_TEMP_SENSOR_OLD)
#define ESPCM_TEMP_SENSOR_AVAILABLE 1
#else
#define ESPCM_TEMP_SENSOR_AVAILABLE 0
#endif

#ifndef ESPCM_HAS_TEMP_SENSOR_NEW
#define ESPCM_HAS_TEMP_SENSOR_NEW 0
#endif

#ifndef ESPCM_HAS_TEMP_SENSOR_OLD
#define ESPCM_HAS_TEMP_SENSOR_OLD 0
#endif

struct CpuMonitorConfig {
    uint32_t sampleIntervalMs = 1000;   // Periodic sampling interval (0 = manual only)
    uint32_t calibrationSamples = 5;    // Baseline windows that represent 100% idle
    size_t historySize = 60;            // Ring buffer depth (0 = disable history)
    bool enablePerCore = true;          // When false, only average is reported
    bool enableTemperature = true;      // When false, skip temperature readings
};

struct CpuUsageSample {
    uint64_t timestampUs = 0;
    float perCore[portNUM_PROCESSORS]{};
    float average = 0.0f;
    float temperatureC = std::numeric_limits<float>::quiet_NaN();
    float temperatureAvgC = std::numeric_limits<float>::quiet_NaN();
};

using CpuSampleCallback = std::function<void(const CpuUsageSample&)>;

class ESPCpuMonitor {
public:
    ESPCpuMonitor();
    ~ESPCpuMonitor();

    bool init(const CpuMonitorConfig &cfg = {});
    void deinit();

    bool isReady() const;
    bool getLastSample(CpuUsageSample &out) const;
    float getLastAverage() const;
    bool getLastTemperature(float &currentC, float &averageC) const;
    std::vector<CpuUsageSample> history() const;

    // Trigger sampling immediately (useful when sampleIntervalMs == 0)
    bool sampleNow(CpuUsageSample &out);

    void onSample(CpuSampleCallback cb);

#if ESPCM_HAS_ARDUINOJSON
    void toJson(const CpuUsageSample &sample, JsonDocument &doc) const;
#endif

private:
    static bool IRAM_ATTR idleHookCore0();
#if portNUM_PROCESSORS > 1
    static bool IRAM_ATTR idleHookCore1();
#endif
    static void timerCallback(void *arg);

    void resetState(const CpuMonitorConfig &cfg);
    bool computeSampleLocked(CpuUsageSample &out);
    bool captureSample(CpuUsageSample &out, std::vector<CpuSampleCallback> &callbacksCopy);
    bool initTemperatureSensor();
    void deinitTemperatureSensor();
    bool readTemperature(float &outC);
    void resetTemperatureState();
    void lock() const;
    void unlock() const;

    static volatile uint64_t s_idleCount[portNUM_PROCESSORS];
    static ESPCpuMonitor *s_instance;

    CpuMonitorConfig config_{};
    uint64_t prevIdle_[portNUM_PROCESSORS];
    float idleBaseline_[portNUM_PROCESSORS];
    CpuUsageSample lastSample_{};
    bool hasSample_ = false;
    bool calibrated_ = false;
    uint32_t calibrationSamplesNeeded_ = 0;
    uint32_t calibrationSamplesDone_ = 0;
    esp_timer_handle_t timer_ = nullptr;
    mutable SemaphoreHandle_t mutex_ = nullptr;
    std::deque<CpuUsageSample> history_;
    std::vector<CpuSampleCallback> callbacks_;
    float lastTemperature_ = std::numeric_limits<float>::quiet_NaN();
    float temperatureAvg_ = std::numeric_limits<float>::quiet_NaN();
    uint32_t temperatureSamples_ = 0;
    bool temperatureEnabled_ = false;
#if ESPCM_HAS_TEMP_SENSOR_NEW
    temperature_sensor_handle_t tempSensor_ = nullptr;
#elif ESPCM_HAS_TEMP_SENSOR_OLD
    bool tempSensorStarted_ = false;
#endif
};

extern ESPCpuMonitor cpuMonitor;
