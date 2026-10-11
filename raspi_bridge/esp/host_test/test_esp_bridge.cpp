// Runs esp_bridge.ino's setup()/loop() on the host against the lines the laptop actually sends,
// and checks the servo channels, drive PWM and replies. Build: see run.sh.
#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"
#include <cassert>
#include <iostream>
unsigned long g_now = 1000;
std::map<int, int> g_pin, g_pwm, g_servo_us;
SerialShim Serial;
WireShim Wire;
bool g_i2c_ok = true;
std::set<int> g_i2c_devices = {0x40};
#include "build/esp_bridge_pre.cpp"

static int fails = 0;
#define CHECK(cond, what) do { if (cond) std::cout << "  ok    " << what << "\n"; \
  else { std::cout << "  FAIL  " << what << "\n"; fails++; } } while (0)

// Feed one line, then run loop() for `ms` of simulated time in 10 ms steps.
void send(const std::string& line, unsigned long ms = 40) {
  for (char c : line + "\n") Serial.in.push_back(c);
  for (unsigned long t = 0; t < ms; t += 10) { loop(); g_now += 10; }
}
void idle(unsigned long ms) { for (unsigned long t = 0; t < ms; t += 10) { loop(); g_now += 10; } }
bool said(const std::string& s) { return Serial.out.find(s) != std::string::npos; }
void clear_out() { Serial.out.clear(); }
// gripper pulses: closed = 1150/1850 us (L/R), open = 2000/1000 us
bool upper_closed() { return g_servo_us[13] == 1150 && g_servo_us[12] == 1850; }
bool lower_closed() { return g_servo_us[9] == 1150 && g_servo_us[8] == 1850; }
bool upper_opened() { return g_servo_us[13] == 2000 && g_servo_us[12] == 1000; }
bool lower_opened() { return g_servo_us[9] == 2000 && g_servo_us[8] == 1000; }

int main() {
  setup();
  std::cout << "boot\n";
  CHECK(upper_closed() && lower_closed(), "both grippers closed at power-on (old 13..8 loop never ran)");
  CHECK(said("INIT: i2c devices: 0x40"), "boot line lists the servo board on the I2C bus");

  std::cout << "laptop's 4-field lines (gamepad_to_arduino_direct.py)\n";
  clear_out(); send("150,FORWARD,NONE,manual");
  CHECK(g_pwm[MOTOR_A_EN] == 150 && g_pwm[MOTOR_B_EN] == 150 && g_pin[MOTOR_A_IN1] == 1, "drive runs at 150 forward");
  clear_out(); send("0,FORWARD,up_detach,manual");
  CHECK(said("ACK:up_detach") && !said("ACK:up_detach,manual"), "up_detach acknowledged as a real command");
  CHECK(upper_opened(), "upper gripper opens immediately");
  CHECK(g_servo_us[11] == 1500, "bar not moved yet (waits delay_to_pose)");
  idle(1600);
  CHECK(g_servo_us[11] == 1266 && g_servo_us[10] == 1733, "bar swings to the up_detach pose 1.5 s later");

  std::cout << "interlock: never both open\n";
  clear_out(); send("0,FORWARD,down_detach,manual");
  CHECK(said("ERR:interlock") && lower_closed(), "down_detach refused while the upper is open; lower stays closed");
  clear_out(); send("0,FORWARD,up_attach,manual"); idle(1600);
  CHECK(said("ACK:up_attach") && upper_closed() && lower_closed() && g_servo_us[11] == 1500, "up_attach: bar parallel, both closed");
  clear_out(); send("0,FORWARD,down_detach,manual");
  CHECK(said("ACK:down_detach") && lower_opened(), "down_detach allowed once the upper is clamped");
  idle(1600); send("0,FORWARD,down_attach,manual"); idle(1600);

  std::cout << "busy: one gripper command at a time\n";
  clear_out(); send("0,FORWARD,up_detach,manual"); send("0,FORWARD,both_detach,manual");
  CHECK(said("ERR:busy:both_detach") && lower_closed(), "second command during the 1.5 s move is refused");
  idle(1600); send("0,FORWARD,up_attach,manual"); idle(1600);

  std::cout << "e-stop works while a servo move is in progress\n";
  send("150,FORWARD,NONE,manual");
  clear_out(); send("150,FORWARD,down_detach,manual", 20);
  send("150,FORWARD,estop,manual", 20);
  CHECK(g_pwm[MOTOR_A_EN] == 0 && g_pwm[MOTOR_B_EN] == 0, "motors stop on estop, mid-sequence (was blocked 1.5 s before)");
  send("200,FORWARD,NONE,manual");
  CHECK(g_pwm[MOTOR_A_EN] == 0, "latched: the 30 Hz drive stream cannot restart the motors");
  idle(1600);
  CHECK(g_servo_us[15] == 1733 && g_servo_us[14] == 1266, "the pending bar move still completes after the e-stop");
  clear_out(); send("0,FORWARD,down_attach,manual"); idle(1600);
  CHECK(said("ACK:down_attach") && lower_closed(), "any command clears the latch (existing recovery path)");

  std::cout << "link watchdog\n";
  send("180,BACKWARD,NONE,manual");
  CHECK(g_pwm[MOTOR_A_EN] == 180, "driving at 180");
  clear_out(); idle(400);
  CHECK(g_pwm[MOTOR_A_EN] == 180, "still driving after 400 ms of silence");
  idle(200);
  CHECK(g_pwm[MOTOR_A_EN] == 0 && said("ERR:link timeout"), "stops after 500 ms with no line from the laptop");
  send("120,FORWARD,NONE,manual");
  CHECK(g_pwm[MOTOR_A_EN] == 120, "resumes when lines arrive again");

  send("120,FORWARD,NONE,manual");
  std::cout << "pins (ESP32-S3)\n";
  CHECK(g_pin[MOTOR_A_IN1] == 1 && g_pin[MOTOR_A_IN2] == 0 && g_pin[MOTOR_B_IN1] == 1 && g_pin[MOTOR_B_IN2] == 0,
        "forward: IN1/IN3 high, IN2/IN4 low");
  {
    bool ok = true;   // S3: GPIO 22-25 don't exist, 26-32 are the flash/PSRAM pins
    for (auto& m : {g_pin, g_pwm}) for (auto& kv : m) if ((kv.first >= 22 && kv.first <= 32) || kv.first > 48) ok = false;
    CHECK(ok, "every pin the firmware drives exists on an ESP32-S3 and is not a flash pin");
  }

  std::cout << "other inputs\n";
  clear_out(); send("0,FORWARD,axon_open,manual");
  CHECK(said("ERR:unknown cmd:axon_open") && !said("ACK:axon_open"), "unimplemented command reported, not fake-ACKed");
  clear_out(); send("0,FORWARD,both_attach"); idle(1600);
  CHECK(said("ACK:both_attach"), "old 3-field lines still work (Pi bridge, dashboard)");
  clear_out(); send("0,FORWARD,NONE,hold_alt");
  CHECK(!said("ERR"), "hold_alt field accepted and ignored (no altitude control on this firmware)");

  std::cout << (fails ? "FAILED: " : "ALL PASSED") << (fails ? std::to_string(fails) : "") << "\n";
  return fails != 0;
}
