#include <Arduino.h>
#include <ESPCpuMonitor.h>
#include <unity.h>

namespace {

CpuMonitorConfig testConfig() {
    CpuMonitorConfig cfg{};
    cfg.sampleIntervalMs = 0;
    cfg.calibrationSamples = 1;
    cfg.historySize = 4;
    cfg.enablePerCore = true;
    cfg.enableTemperature = false;
    cfg.smoothingMode = CpuSmoothingMode::None;
    return cfg;
}

void test_deinit_is_safe_before_init() {
    ESPCpuMonitor monitor;
    TEST_ASSERT_FALSE(monitor.isInitialized());

    monitor.deinit();
    TEST_ASSERT_FALSE(monitor.isInitialized());
}

void test_deinit_is_idempotent() {
    ESPCpuMonitor monitor;
    TEST_ASSERT_TRUE(monitor.init(testConfig()));
    TEST_ASSERT_TRUE(monitor.isInitialized());

    monitor.deinit();
    TEST_ASSERT_FALSE(monitor.isInitialized());

    monitor.deinit();
    TEST_ASSERT_FALSE(monitor.isInitialized());
}

void test_reinit_after_deinit() {
    ESPCpuMonitor monitor;
    TEST_ASSERT_TRUE(monitor.init(testConfig()));
    TEST_ASSERT_TRUE(monitor.isInitialized());

    monitor.deinit();
    TEST_ASSERT_FALSE(monitor.isInitialized());

    TEST_ASSERT_TRUE(monitor.init(testConfig()));
    TEST_ASSERT_TRUE(monitor.isInitialized());
    monitor.deinit();
}

void test_destructor_deinits_active_instance() {
    {
        ESPCpuMonitor first;
        TEST_ASSERT_TRUE(first.init(testConfig()));
        TEST_ASSERT_TRUE(first.isInitialized());
    }

    ESPCpuMonitor second;
    TEST_ASSERT_TRUE(second.init(testConfig()));
    TEST_ASSERT_TRUE(second.isInitialized());
    second.deinit();
}

}  // namespace

void setUp() {}
void tearDown() {}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_deinit_is_safe_before_init);
    RUN_TEST(test_deinit_is_idempotent);
    RUN_TEST(test_reinit_after_deinit);
    RUN_TEST(test_destructor_deinits_active_instance);
    UNITY_END();
}

void loop() {
    delay(1000);
}
