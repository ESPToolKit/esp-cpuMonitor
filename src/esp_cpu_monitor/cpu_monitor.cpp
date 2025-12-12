#include "cpu_monitor.h"

#include <esp_log.h>
#include <cmath>
#include <type_traits>

static const char *TAG = "ESPCpuMonitor";

volatile uint64_t ESPCpuMonitor::s_idleCount[portNUM_PROCESSORS] = {};
ESPCpuMonitor *ESPCpuMonitor::s_instance = nullptr;

ESPCpuMonitor::ESPCpuMonitor() {
    resetState(CpuMonitorConfig{});
}

ESPCpuMonitor::~ESPCpuMonitor() {
    deinit();
}

template <typename TResult>
static bool idleHookRegisteredSuccessfully(const TResult &res) {
#if defined(ESP_OK)
    if constexpr (std::is_same<TResult, esp_err_t>::value) {
        return res == ESP_OK;
    }
#endif
    return static_cast<bool>(res);
}

template <typename TResult>
static int idleHookErrorCode(const TResult &res) {
#if defined(ESP_OK)
    if constexpr (std::is_same<TResult, esp_err_t>::value) {
        return static_cast<int>(res);
    }
#endif
    return static_cast<bool>(res) ? 0 : -1;
}

void ESPCpuMonitor::resetState(const CpuMonitorConfig &cfg) {
    config_ = cfg;
    calibrationSamplesNeeded_ = config_.calibrationSamples == 0 ? 1 : config_.calibrationSamples;
    calibrationSamplesDone_ = 0;
    hasSample_ = false;
    calibrated_ = false;
    history_.clear();
    resetTemperatureState();
    temperatureEnabled_ = false;
#if ESPCM_HAS_TEMP_SENSOR_NEW
    tempSensor_ = nullptr;
#elif ESPCM_HAS_TEMP_SENSOR_OLD
    tempSensorStarted_ = false;
#endif

    for (int i = 0; i < portNUM_PROCESSORS; ++i) {
        prevIdle_[i] = 0;
        idleBaseline_[i] = 0.0f;
        lastSample_.perCore[i] = 0.0f;
        s_idleCount[i] = 0;
    }
    lastSample_.average = 0.0f;
    lastSample_.temperatureC = std::numeric_limits<float>::quiet_NaN();
    lastSample_.temperatureAvgC = std::numeric_limits<float>::quiet_NaN();
}

void ESPCpuMonitor::lock() const {
    if (mutex_) {
        xSemaphoreTake(mutex_, portMAX_DELAY);
    }
}

void ESPCpuMonitor::unlock() const {
    if (mutex_) {
        xSemaphoreGive(mutex_);
    }
}

bool ESPCpuMonitor::init(const CpuMonitorConfig &cfg) {
    if (s_instance != nullptr) {
        ESP_LOGW(TAG, "ESPCpuMonitor already running");
        return false;
    }

    resetState(cfg);

    if (!mutex_) {
        mutex_ = xSemaphoreCreateMutex();
        if (!mutex_) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return false;
        }
    }

    bool hook0 = false;
    bool hook1 = false;

#if portNUM_PROCESSORS == 1
    const auto res0 = esp_register_freertos_idle_hook(&ESPCpuMonitor::idleHookCore0);
    hook0 = idleHookRegisteredSuccessfully(res0);
    if (!hook0) {
        ESP_LOGE(TAG, "Failed to register idle hook for core0 (err=%d)", idleHookErrorCode(res0));
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
        return false;
    }
#else
    const auto res0 = esp_register_freertos_idle_hook_for_cpu(&ESPCpuMonitor::idleHookCore0, 0);
    hook0 = idleHookRegisteredSuccessfully(res0);
    if (!hook0) {
        ESP_LOGE(TAG, "Failed to register idle hook for core0 (err=%d)", idleHookErrorCode(res0));
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
        return false;
    }
    const auto res1 = esp_register_freertos_idle_hook_for_cpu(&ESPCpuMonitor::idleHookCore1, 1);
    hook1 = idleHookRegisteredSuccessfully(res1);
    if (!hook1) {
        ESP_LOGE(TAG, "Failed to register idle hook for core1 (err=%d)", idleHookErrorCode(res1));
        esp_deregister_freertos_idle_hook_for_cpu(&ESPCpuMonitor::idleHookCore0, 0);
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
        return false;
    }
#endif

    s_instance = this;

    if (config_.sampleIntervalMs > 0) {
        esp_timer_create_args_t args = {};
        args.callback = &ESPCpuMonitor::timerCallback;
        args.arg = this;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = "cpu_monitor";

        esp_err_t err = esp_timer_create(&args, &timer_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_timer_create failed: %d", static_cast<int>(err));
            deinit();
            return false;
        }

        err = esp_timer_start_periodic(timer_, config_.sampleIntervalMs * 1000);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_timer_start_periodic failed: %d", static_cast<int>(err));
            deinit();
            return false;
        }
    }

    if (config_.enableTemperature) {
        temperatureEnabled_ = initTemperatureSensor();
        if (!temperatureEnabled_) {
            ESP_LOGW(TAG, "CPU temperature sensor unavailable; temperature readings disabled");
        }
    }

    ESP_LOGI(TAG, "ESPCpuMonitor started (period=%ums, calibration=%u)",
             config_.sampleIntervalMs, calibrationSamplesNeeded_);
    return true;
}

void ESPCpuMonitor::deinit() {
    if (s_instance != this) {
        return;
    }

    if (timer_) {
        esp_timer_stop(timer_);
        esp_timer_delete(timer_);
        timer_ = nullptr;
    }

    deinitTemperatureSensor();

#if portNUM_PROCESSORS == 1
    esp_deregister_freertos_idle_hook(&ESPCpuMonitor::idleHookCore0);
#else
    esp_deregister_freertos_idle_hook_for_cpu(&ESPCpuMonitor::idleHookCore0, 0);
    esp_deregister_freertos_idle_hook_for_cpu(&ESPCpuMonitor::idleHookCore1, 1);
#endif

    if (mutex_) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }

    s_instance = nullptr;
    calibrated_ = false;
    hasSample_ = false;
    history_.clear();
}

bool ESPCpuMonitor::isReady() const {
    lock();
    bool ready = calibrated_ && hasSample_;
    unlock();
    return ready;
}

bool ESPCpuMonitor::getLastSample(CpuUsageSample &out) const {
    if (!isReady()) {
        return false;
    }
    lock();
    out = lastSample_;
    unlock();
    return true;
}

float ESPCpuMonitor::getLastAverage() const {
    CpuUsageSample sample{};
    if (getLastSample(sample)) {
        return sample.average;
    }
    return -1.0f;
}

bool ESPCpuMonitor::getLastTemperature(float &currentC, float &averageC) const {
    if (!config_.enableTemperature) {
        return false;
    }
    CpuUsageSample sample{};
    if (!getLastSample(sample)) {
        return false;
    }
    if (std::isnan(sample.temperatureC) || std::isnan(sample.temperatureAvgC)) {
        return false;
    }
    currentC = sample.temperatureC;
    averageC = sample.temperatureAvgC;
    return true;
}

std::vector<CpuUsageSample> ESPCpuMonitor::history() const {
    lock();
    std::vector<CpuUsageSample> out(history_.begin(), history_.end());
    unlock();
    return out;
}

bool ESPCpuMonitor::sampleNow(CpuUsageSample &out) {
    if (s_instance != this) {
        ESP_LOGE(TAG, "Call init() before sampleNow()");
        return false;
    }

    std::vector<CpuSampleCallback> callbacks;
    bool ready = captureSample(out, callbacks);
    for (const auto &cb : callbacks) {
        if (cb) {
            cb(out);
        }
    }
    return ready;
}

void ESPCpuMonitor::onSample(CpuSampleCallback cb) {
    if (!cb) {
        return;
    }
    lock();
    callbacks_.push_back(cb);
    unlock();
}

bool IRAM_ATTR ESPCpuMonitor::idleHookCore0() {
    s_idleCount[0]++;
    return true;
}

#if portNUM_PROCESSORS > 1
bool IRAM_ATTR ESPCpuMonitor::idleHookCore1() {
    s_idleCount[1]++;
    return true;
}
#endif

void ESPCpuMonitor::timerCallback(void *arg) {
    auto *self = static_cast<ESPCpuMonitor *>(arg);
    if (!self) {
        return;
    }
    CpuUsageSample sample{};
    std::vector<CpuSampleCallback> callbacks;
    if (self->captureSample(sample, callbacks)) {
        for (const auto &cb : callbacks) {
            if (cb) {
                cb(sample);
            }
        }
    }
}

bool ESPCpuMonitor::captureSample(CpuUsageSample &out, std::vector<CpuSampleCallback> &callbacksCopy) {
    bool ready = false;
    lock();
    ready = computeSampleLocked(out);
    if (ready) {
        lastSample_ = out;
        hasSample_ = true;

        if (config_.historySize > 0) {
            history_.push_back(out);
            if (history_.size() > config_.historySize) {
                history_.pop_front();
            }
        }

        callbacksCopy = callbacks_;
    }
    unlock();
    return ready;
}

bool ESPCpuMonitor::computeSampleLocked(CpuUsageSample &out) {
    float perCoreUsage[portNUM_PROCESSORS] = {};
    float avg = 0.0f;
    uint64_t nowUs = esp_timer_get_time();

    for (int i = 0; i < portNUM_PROCESSORS; ++i) {
        uint64_t currentIdle = s_idleCount[i];
        uint64_t deltaIdle = currentIdle - prevIdle_[i];

        if (!calibrated_) {
            idleBaseline_[i] += static_cast<float>(deltaIdle);
        } else {
            float baseline = idleBaseline_[i] <= 0.0f ? 1.0f : idleBaseline_[i];
            float idleRatio = baseline > 0.0f ? static_cast<float>(deltaIdle) / baseline : 0.0f;
            float usage = 100.0f * (1.0f - idleRatio);
            if (usage < 0.0f) usage = 0.0f;
            if (usage > 100.0f) usage = 100.0f;
            perCoreUsage[i] = usage;
            avg += usage;
        }

        prevIdle_[i] = currentIdle;
    }

    if (!calibrated_) {
        calibrationSamplesDone_++;
        if (calibrationSamplesDone_ >= calibrationSamplesNeeded_) {
            for (int i = 0; i < portNUM_PROCESSORS; ++i) {
                idleBaseline_[i] /= static_cast<float>(calibrationSamplesNeeded_);
                if (idleBaseline_[i] < 1.0f) {
                    idleBaseline_[i] = 1.0f;
                }
            }
            calibrated_ = true;
            ESP_LOGI(TAG, "CPU usage baseline calibrated after %u samples", calibrationSamplesNeeded_);
        }
        return false;
    }

    avg /= static_cast<float>(portNUM_PROCESSORS);

    out.timestampUs = nowUs;
    for (int i = 0; i < portNUM_PROCESSORS; ++i) {
        out.perCore[i] = config_.enablePerCore ? perCoreUsage[i] : avg;
    }
    out.average = avg;

    float temperature = std::numeric_limits<float>::quiet_NaN();
    float temperatureAvg = std::numeric_limits<float>::quiet_NaN();
    if (readTemperature(temperature)) {
        if (temperatureSamples_ == 0) {
            temperatureAvg_ = temperature;
        } else {
            temperatureAvg_ += (temperature - temperatureAvg_) / static_cast<float>(temperatureSamples_ + 1);
        }
        temperatureSamples_++;
        lastTemperature_ = temperature;
        temperatureAvg = temperatureAvg_;
    } else if (temperatureSamples_ > 0) {
        temperatureAvg = temperatureAvg_;
        temperature = lastTemperature_;
    }
    out.temperatureC = temperature;
    out.temperatureAvgC = temperatureAvg;
    return true;
}

#if ESPCM_HAS_ARDUINOJSON
void ESPCpuMonitor::toJson(const CpuUsageSample &sample, JsonDocument &doc) const {
    doc["ts"] = sample.timestampUs;
    auto coreArr = doc["core"].to<JsonArray>();
    for (size_t i = 0; i < portNUM_PROCESSORS; ++i) {
        coreArr.add(sample.perCore[i]);
    }
    doc["avg"] = sample.average;
    if (!std::isnan(sample.temperatureC)) {
        doc["temp"] = sample.temperatureC;
    }
    if (!std::isnan(sample.temperatureAvgC)) {
        doc["tempAvg"] = sample.temperatureAvgC;
    }
}
#endif

bool ESPCpuMonitor::initTemperatureSensor() {
#if ESPCM_TEMP_SENSOR_AVAILABLE && ESPCM_HAS_TEMP_SENSOR_NEW
    temperature_sensor_config_t cfg{};
#if defined(TEMPERATURE_SENSOR_CONFIG_DEFAULT) && defined(TEMPERATURE_SENSOR_CLK_SRC_DEFAULT)
    cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
#elif defined(TEMPERATURE_SENSOR_CONFIG_DEFAULT)
    // Some Arduino ESP32 builds expose the macro but not the clock source enum; set only the range.
    cfg.range_min = -10;
    cfg.range_max = 80;
#else
    cfg.range_min = -10;
    cfg.range_max = 80;
#endif
    esp_err_t err = temperature_sensor_install(&cfg, &tempSensor_);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "temperature_sensor_install failed: %d", static_cast<int>(err));
        tempSensor_ = nullptr;
        return false;
    }
    err = temperature_sensor_enable(tempSensor_);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "temperature_sensor_enable failed: %d", static_cast<int>(err));
        temperature_sensor_uninstall(tempSensor_);
        tempSensor_ = nullptr;
        return false;
    }
    return true;
#elif ESPCM_TEMP_SENSOR_AVAILABLE && ESPCM_HAS_TEMP_SENSOR_OLD
    temp_sensor_config_t cfg = TSENS_CONFIG_DEFAULT();
    esp_err_t err = temp_sensor_set_config(cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "temp_sensor_set_config failed: %d", static_cast<int>(err));
        return false;
    }
    err = temp_sensor_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "temp_sensor_start failed: %d", static_cast<int>(err));
        return false;
    }
    tempSensorStarted_ = true;
    return true;
#else  // ESPCM_TEMP_SENSOR_AVAILABLE && ESPCM_HAS_TEMP_SENSOR_NEW
    return false;
#endif
}

void ESPCpuMonitor::deinitTemperatureSensor() {
#if ESPCM_TEMP_SENSOR_AVAILABLE && ESPCM_HAS_TEMP_SENSOR_NEW
    if (tempSensor_) {
        temperature_sensor_disable(tempSensor_);
        temperature_sensor_uninstall(tempSensor_);
        tempSensor_ = nullptr;
    }
#elif ESPCM_TEMP_SENSOR_AVAILABLE && ESPCM_HAS_TEMP_SENSOR_OLD
    if (tempSensorStarted_) {
        temp_sensor_stop();
        tempSensorStarted_ = false;
    }
#endif
    temperatureEnabled_ = false;
    resetTemperatureState();
}

bool ESPCpuMonitor::readTemperature(float &outC) {
    if (!config_.enableTemperature || !temperatureEnabled_) {
        return false;
    }
#if ESPCM_TEMP_SENSOR_AVAILABLE && ESPCM_HAS_TEMP_SENSOR_NEW
    if (!tempSensor_) {
        return false;
    }
    float val = 0.0f;
    if (temperature_sensor_get_celsius(tempSensor_, &val) == ESP_OK) {
        outC = val;
        return true;
    }
#elif ESPCM_TEMP_SENSOR_AVAILABLE && ESPCM_HAS_TEMP_SENSOR_OLD
    float val = 0.0f;
    if (temp_sensor_read_celsius(&val) == ESP_OK) {
        outC = val;
        return true;
    }
#endif
    return false;
}

void ESPCpuMonitor::resetTemperatureState() {
    lastTemperature_ = std::numeric_limits<float>::quiet_NaN();
    temperatureAvg_ = std::numeric_limits<float>::quiet_NaN();
    temperatureSamples_ = 0;
}
