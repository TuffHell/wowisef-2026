#pragma once
#include <cstdint>
#include "Adafruit_Sensor.h"
extern bool g_bno_present;
struct Adafruit_BNO055 {
  Adafruit_BNO055(int, uint8_t) {}
  bool begin() { return g_bno_present; }
  void setExtCrystalUse(bool) {}
  void getEvent(sensors_event_t* e) { *e = sensors_event_t(); }   // level: no tilt
};
