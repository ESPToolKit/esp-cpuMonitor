#include "cpu_monitor.h"

#include <esp_log.h>

static const char *TAG = "ESPCpuMonitor";

volatile uint64_t ESPCpuMonitor::s_idleCount[portNUM_PROCESSORS] = {};
ESPCpuMonitor *ESPCpuMonitor::s_instance = nullptr;

ESPCpuMonitor cpuMonitor;

ESPCpuMonitor::ESPCpuMonitor() {
    resetState(CpuMonitorConfig{});
}

ESPCpuMonitor::~ESPCpuMonitor() {
    deinit();
}

void ESPCpuMonitor::resetState(const CpuMonitorConfig &cfg) {
    config_ = cfg;
    calibrationSamplesNeeded_ = config_.calibrationSamples == 0 ? 1 : config_.calibrationSamples;
    calibrationSamplesDone_ = 0;
    hasSample_ = false;
    calibrated_ = false;
    history_.clear();

    for (int i = 0; i < portNUM_PROCESSORS; ++i) {
        prevIdle_[i] = 0;
        idleBaseline_[i] = 0.0f;
        lastSample_.perCore[i] = 0.0f;
        s_idleCount[i] = 0;
    }
    lastSample_.average = 0.0f;
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
    hook0 = esp_register_freertos_idle_hook(&ESPCpuMonitor::idleHookCore0);
    if (!hook0) {
        ESP_LOGE(TAG, "Failed to register idle hook for core0");
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
        return false;
    }
#else
    hook0 = esp_register_freertos_idle_hook_for_cpu(&ESPCpuMonitor::idleHookCore0, 0);
    if (!hook0) {
        ESP_LOGE(TAG, "Failed to register idle hook for core0");
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
        return false;
    }
    hook1 = esp_register_freertos_idle_hook_for_cpu(&ESPCpuMonitor::idleHookCore1, 1);
    if (!hook1) {
        ESP_LOGE(TAG, "Failed to register idle hook for core1");
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

    ESP_LOGI(TAG, "ESPCpuMonitor started (period=%ums, calibration=%u)",
             config_.sampleIntervalMs, calibrationSamplesNeeded_);
    return true;
}

void ESPCpuMonitor::deinit() {
    if (timer_) {
        esp_timer_stop(timer_);
        esp_timer_delete(timer_);
        timer_ = nullptr;
    }

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
}
#endif
