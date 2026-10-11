#pragma once
#include <set>
// g_i2c_ok: whether Wire.begin() succeeds (false = pins that don't exist); g_i2c_devices: who answers
extern bool g_i2c_ok;
extern std::set<int> g_i2c_devices;
struct WireShim {
  int addr = 0;
  bool begin(int, int) { return g_i2c_ok; }
  void beginTransmission(int a) { addr = a; }
  int endTransmission() { return g_i2c_ok && g_i2c_devices.count(addr) ? 0 : 2; }
};
extern WireShim Wire;
