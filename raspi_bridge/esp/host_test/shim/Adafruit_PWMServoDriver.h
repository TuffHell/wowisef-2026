#pragma once
#include <map>
// g_servo[board address][channel] = last pulse written, in microseconds
extern std::map<int, std::map<int, int>> g_servo;
struct Adafruit_PWMServoDriver {
  int addr;
  Adafruit_PWMServoDriver(int a) : addr(a) {}
  bool begin() { return true; } void reset() {} void setPWMFreq(float) {}
  void writeMicroseconds(int ch, int us) { g_servo[addr][ch] = us; }
};
