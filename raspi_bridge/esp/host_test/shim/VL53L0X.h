#pragma once
#include <cstdint>
struct VL53L0X { bool init() { return true; } void setAddress(uint8_t) {} void startContinuous() {} void setTimeout(uint16_t) {}
  uint16_t readRangeContinuousMillimeters() { return 100; } bool timeoutOccurred() { return false; }
  bool setMeasurementTimingBudget(uint32_t) { return true; } };
