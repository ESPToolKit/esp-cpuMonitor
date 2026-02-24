# Host-side tests are not available because ESPCpuMonitor depends on
# ESP32 FreeRTOS idle hooks and esp_timer. Run on-device tests in
# test/test_cpu_monitor with PlatformIO/Arduino.
message(STATUS "ESPCpuMonitor: host tests are disabled.")
