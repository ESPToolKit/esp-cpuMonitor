#include <Arduino.h>
#include <ESPCpuMonitor.h>
#include <cmath>
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

void spinWork(uint32_t durationMicros) {
	const uint32_t start = micros();
	while ((uint32_t)(micros() - start) < durationMicros) {
		asm volatile("nop");
	}
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

void test_start_measure_before_init_is_invalid() {
	ESPCpuMonitor monitor;
	CpuMeasureToken token = monitor.startMeasure();
	TEST_ASSERT_FALSE(token.valid);
}

void test_stop_measure_with_invalid_token_is_invalid() {
	ESPCpuMonitor monitor;
	TEST_ASSERT_TRUE(monitor.init(testConfig()));
	CpuMeasure result = monitor.stopMeasure(CpuMeasureToken{});
	TEST_ASSERT_FALSE(result.valid);
	monitor.deinit();
}

void test_measure_before_calibration_has_only_timing() {
	ESPCpuMonitor monitor;
	CpuMonitorConfig cfg = testConfig();
	cfg.calibrationSamples = 3;
	TEST_ASSERT_TRUE(monitor.init(cfg));

	CpuMeasureToken token = monitor.startMeasure();
	TEST_ASSERT_TRUE(token.valid);
	delay(2);

	CpuMeasure result = monitor.stopMeasure(token);
	TEST_ASSERT_TRUE(result.valid);
	TEST_ASSERT_FALSE(result.hasCpuData);
	TEST_ASSERT_TRUE(result.durationUs > 0);
	TEST_ASSERT_TRUE(result.durationMs > 0.0);
	TEST_ASSERT_TRUE(result.durationSec > 0.0);
	TEST_ASSERT_TRUE(std::isnan(result.averageUsage));
	for (size_t i = 0; i < portNUM_PROCESSORS; ++i) {
		TEST_ASSERT_TRUE(std::isnan(result.perCoreUsage[i]));
	}

	monitor.deinit();
}

void test_measure_after_calibration_has_cpu_data() {
	ESPCpuMonitor monitor;
	CpuMonitorConfig cfg = testConfig();
	cfg.calibrationSamples = 1;
	TEST_ASSERT_TRUE(monitor.init(cfg));

	CpuUsageSample sample{};
	TEST_ASSERT_FALSE(monitor.sampleNow(sample)); // completes calibration

	CpuMeasureToken token = monitor.startMeasure();
	TEST_ASSERT_TRUE(token.valid);
	spinWork(4000);
	delay(2);

	CpuMeasure result = monitor.stopMeasure(token);
	TEST_ASSERT_TRUE(result.valid);
	TEST_ASSERT_TRUE(result.hasCpuData);
	TEST_ASSERT_TRUE(result.durationUs > 0);
	TEST_ASSERT_FALSE(std::isnan(result.averageUsage));
	TEST_ASSERT_TRUE(result.averageUsage >= 0.0f);
	TEST_ASSERT_TRUE(result.averageUsage <= 100.0f);
	for (size_t i = 0; i < portNUM_PROCESSORS; ++i) {
		TEST_ASSERT_FALSE(std::isnan(result.perCoreUsage[i]));
		TEST_ASSERT_TRUE(result.perCoreUsage[i] >= 0.0f);
		TEST_ASSERT_TRUE(result.perCoreUsage[i] <= 100.0f);
	}

	monitor.deinit();
}

void test_overlapping_measure_tokens_are_independent() {
	ESPCpuMonitor monitor;
	CpuMonitorConfig cfg = testConfig();
	cfg.calibrationSamples = 1;
	TEST_ASSERT_TRUE(monitor.init(cfg));

	CpuUsageSample sample{};
	TEST_ASSERT_FALSE(monitor.sampleNow(sample)); // completes calibration

	CpuMeasureToken token1 = monitor.startMeasure();
	TEST_ASSERT_TRUE(token1.valid);
	delay(2);
	CpuMeasureToken token2 = monitor.startMeasure();
	TEST_ASSERT_TRUE(token2.valid);
	delay(2);

	CpuMeasure result2 = monitor.stopMeasure(token2);
	TEST_ASSERT_TRUE(result2.valid);
	TEST_ASSERT_TRUE(result2.durationUs > 0);

	delay(2);
	CpuMeasure result1 = monitor.stopMeasure(token1);
	TEST_ASSERT_TRUE(result1.valid);
	TEST_ASSERT_TRUE(result1.durationUs > 0);
	TEST_ASSERT_TRUE(result1.durationUs >= result2.durationUs);
	TEST_ASSERT_TRUE(result1.startedUs <= result2.startedUs);
	TEST_ASSERT_TRUE(result1.hasCpuData);
	TEST_ASSERT_TRUE(result2.hasCpuData);

	monitor.deinit();
}

} // namespace

void setUp() {
}
void tearDown() {
}

void setup() {
	delay(2000);
	UNITY_BEGIN();
	RUN_TEST(test_deinit_is_safe_before_init);
	RUN_TEST(test_deinit_is_idempotent);
	RUN_TEST(test_reinit_after_deinit);
	RUN_TEST(test_destructor_deinits_active_instance);
	RUN_TEST(test_start_measure_before_init_is_invalid);
	RUN_TEST(test_stop_measure_with_invalid_token_is_invalid);
	RUN_TEST(test_measure_before_calibration_has_only_timing);
	RUN_TEST(test_measure_after_calibration_has_cpu_data);
	RUN_TEST(test_overlapping_measure_tokens_are_independent);
	UNITY_END();
}

void loop() {
	delay(1000);
}
