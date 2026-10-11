#pragma once
struct WireShim { void begin(int, int) {} void beginTransmission(int) {} int endTransmission() { return 0; } };
extern WireShim Wire;
