#pragma once
#include <map>
extern std::map<int, int> g_servo_us;
struct Adafruit_PWMServoDriver { Adafruit_PWMServoDriver(int) {} bool begin() { return true; } void reset() {}
  void setPWMFreq(float) {} void writeMicroseconds(int ch, int us) { g_servo_us[ch] = us; } };
