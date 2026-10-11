#pragma once
#include <cstdint>
// g_ground_mm: what the ground-facing VL53L1X reads
extern uint16_t g_ground_mm;
struct VL53L1X {
  enum DistanceMode { Short, Medium, Long };
  enum RangeStatus { RangeValid = 0, SignalFail = 2 };
  struct { RangeStatus range_status = RangeValid; } ranging_data;
  bool init() { return true; } void setAddress(uint8_t) {} void setTimeout(uint16_t) {}
  bool setDistanceMode(DistanceMode) { return true; } bool setMeasurementTimingBudget(uint32_t) { return true; }
  void startContinuous(uint32_t) {} bool dataReady() { return true; } bool timeoutOccurred() { return false; }
  uint16_t read(bool) { return g_ground_mm; }
};
