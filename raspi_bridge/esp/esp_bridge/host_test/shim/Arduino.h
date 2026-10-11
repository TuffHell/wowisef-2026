// Minimal host stand-ins for the Arduino/ESP32 APIs esp_bridge.ino uses, so its command logic
// can run on a Mac. Serial input is a queue the test fills; output and pin writes are recorded.
#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>
#include <string>
#include <map>
#include <deque>
#include <cstdio>
#define ESP_ARDUINO_VERSION_MAJOR 3
#define HIGH 1
#define LOW 0
#define OUTPUT 1
using std::isnan; using std::isinf;
extern unsigned long g_now;
inline unsigned long millis() { return g_now; }
inline void delay(unsigned long ms) { g_now += ms; }
extern std::map<int, int> g_pin, g_pwm;
inline void pinMode(int, int) {}
inline void digitalWrite(int p, int v) { g_pin[p] = v; }
inline bool ledcAttach(int, int, int) { return true; }
inline void ledcWrite(int pin, int duty) { g_pwm[pin] = duty; }

class String {
 public:
  std::string s;
  String() {}
  String(const char* c) : s(c) {}
  String(const std::string& x) : s(x) {}
  unsigned length() const { return s.size(); }
  const char* c_str() const { return s.c_str(); }
  int indexOf(char c, unsigned from = 0) const { auto i = s.find(c, from); return i == std::string::npos ? -1 : (int)i; }
  String substring(unsigned a) const { return a >= s.size() ? String() : String(s.substr(a)); }
  String substring(unsigned a, unsigned b) const { return a >= s.size() ? String() : String(s.substr(a, b - a)); }
  bool startsWith(const char* p) const { return s.rfind(p, 0) == 0; }
  void trim() { size_t a = s.find_first_not_of(" \t\r\n"); size_t b = s.find_last_not_of(" \t\r\n");
                s = a == std::string::npos ? "" : s.substr(a, b - a + 1); }
  long toInt() const { return atol(s.c_str()); }
  bool operator==(const char* o) const { return s == o; }
  bool operator!=(const char* o) const { return s != o; }
  bool operator==(const String& o) const { return s == o.s; }
};

struct SerialShim {
  std::deque<char> in; std::string out;
  void begin(long) {} void setTimeout(long) {} size_t setRxBufferSize(size_t n) { return n; }
  int available() { return (int)in.size(); }
  String readStringUntil(char t) { std::string r; while (!in.empty()) { char c = in.front(); in.pop_front(); if (c == t) break; r += c; } return String(r); }
  void print(const char* x) { out += x; } void print(const String& x) { out += x.s; }
  void print(long x) { out += std::to_string(x); } void print(int x) { out += std::to_string(x); }
  void print(unsigned x) { out += std::to_string(x); } void print(unsigned long x) { out += std::to_string(x); }
  template <class T> void println(const T& x) { print(x); out += "\n"; }
  void println() { out += "\n"; }
};
extern SerialShim Serial;
