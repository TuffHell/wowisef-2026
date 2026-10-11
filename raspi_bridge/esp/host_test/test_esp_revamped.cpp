// Runs esp_revamped.ino's setup()/loop() on the host against the lines gamepad_to_arduino_direct.py
// sends (R2 held = hold_alt on every line), with a stand-in ground sensor, and checks the altitude
// hold, the e-stop, the link watchdog and the gripper rules. Build: run.sh.
#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"
#include <iostream>
unsigned long g_now = 1000;
std::map<int, int> g_pin, g_pwm;
std::map<int, std::map<int, int>> g_servo;
SerialShim Serial;
WireShim Wire;
bool g_i2c_ok = true;
std::set<int> g_i2c_devices = {0x28, 0x29, 0x40, 0x60};   // BNO055, VL53L1X, the two PCA9685 boards
bool g_bno_present = true;
uint16_t g_ground_mm = 500;
#include "build/esp_revamped_pre.cpp"

int grip(int ch) { return g_servo[GRIPPER_HUB_ADDR][ch]; }   // gripper board, 0x60
int bar(int ch) { return g_servo[SERVO_HUB_ADDR][ch]; }      // bar + Axon board, 0x40
// Grippers only ever written on 0x60 channels 0-3; bars and Axon only on 0x40
bool boards_kept_apart() {
  for (auto& kv : g_servo[GRIPPER_HUB_ADDR]) if (kv.first > 3) return false;
  for (auto& kv : g_servo[SERVO_HUB_ADDR]) if (kv.first <= 3) return false;
  return !g_servo[GRIPPER_HUB_ADDR].empty() && !g_servo[SERVO_HUB_ADDR].empty();
}

static int fails = 0;
#define CHECK(cond, what) do { if (cond) std::cout << "  ok    " << what << "\n"; \
  else { std::cout << "  FAIL  " << what << "\n"; fails++; } } while (0)

// The laptop streams at 30 Hz; send `line` every 30 ms for `ms`, running loop() every 10 ms
void stream(const std::string& line, unsigned long ms) {
  for (unsigned long t = 0; t < ms; t += 10) {
    if (t % 30 == 0) for (char c : line + "\n") Serial.in.push_back(c);
    loop(); g_now += 10;
  }
}
void idle(unsigned long ms) { for (unsigned long t = 0; t < ms; t += 10) { loop(); g_now += 10; } }
bool said(const std::string& s) { return Serial.out.find(s) != std::string::npos; }
int drive() { return g_pwm[MOTOR_A_EN]; }

int main() {
  setup();
  std::cout << "boot\n";
  CHECK(upperGripperState == GRIPPER_CLOSED && lowerGripperState == GRIPPER_CLOSED &&
        grip(chServoUpL) == 1150 && grip(chServoDownL) == 1150, "both grippers closed at power-on");
  CHECK(said("INIT: i2c devices: 0x28 0x29 0x40 0x60") && !said("NOT FOUND"), "boot lists every I2C device, both servo boards included");
  stream("0,FORWARD,NONE,manual", 4000);   // let the ground filter settle on 500 mm

  std::cout << "altitude hold (R2)\n";
  stream("0,FORWARD,NONE,hold_alt", 300);
  CHECK(currentMode == MODE_HOLD_ALTITUDE && fabs(targetAltitude_mm - 500) < 2, "R2 enters hold at the current height (500 mm)");
  g_ground_mm = 300;                        // the robot slips 200 mm while R2 is still held
  stream("0,FORWARD,NONE,hold_alt", 1500);
  CHECK(fabs(targetAltitude_mm - 500) < 2, "target stays at 500 mm while R2 is held (was reset 30x/s)");
  CHECK(drive() >= MIN_HOLD_SPEED && current_direction == FORWARD, "controller climbs back toward the target");
  stream("120,FORWARD,NONE,manual", 100);
  CHECK(currentMode == MODE_MANUAL && drive() == 120, "releasing R2 returns to stick control (used to stay in hold)");

  std::cout << "e-stop in hold\n";
  stream("0,FORWARD,NONE,hold_alt", 300);   // press R2 again: hold at ~300 mm
  g_ground_mm = 100;
  stream("0,FORWARD,NONE,hold_alt", 1500);
  CHECK(drive() > 0, "holding: motors running");
  stream("0,FORWARD,estop,hold_alt", 30);
  stream("0,FORWARD,NONE,hold_alt", 1000);  // R2 still held, PS released
  CHECK(drive() == 0 && latched && currentMode == MODE_MANUAL,
        "motors stay stopped after the e-stop (the hold controller restarted them before)");
  CHECK(latched, "holding R2 does not clear the latch (it used to)");
  stream("0,FORWARD,both_attach,manual", 30); idle(delay_to_pose + 100);
  CHECK(!latched, "a button command clears the latch (existing recovery path)");
  g_ground_mm = 500; stream("0,FORWARD,NONE,manual", 3000);

  std::cout << "link watchdog\n";
  stream("0,FORWARD,NONE,hold_alt", 300);
  g_ground_mm = 300; stream("0,FORWARD,NONE,hold_alt", 1500);
  CHECK(drive() > 0 && currentMode == MODE_HOLD_ALTITUDE, "holding and driving");
  Serial.out.clear(); idle(600);
  CHECK(drive() == 0 && currentMode == MODE_MANUAL && said("ERR:link timeout"),
        "laptop silent for 500 ms: motors stop and hold is cancelled");
  stream("150,BACKWARD,NONE,manual", 100);
  CHECK(drive() == 150 && current_direction == BACKWARD, "stick control resumes when lines return");
  g_ground_mm = 500; stream("0,FORWARD,NONE,manual", 3000);

  std::cout << "modes that can't work are refused\n";
  bno_ready = false; Serial.out.clear();
  stream("200,FORWARD,NONE,hold_alt", 100);
  CHECK(currentMode == MODE_MANUAL && said("ERR:hold_alt needs the BNO055"),
        "no IMU: hold refused instead of ignoring the stick with the motors running");
  bno_ready = true; stream("0,FORWARD,NONE,manual", 100);
  Serial.out.clear(); stream("0,FORWARD,NONE,set_alt", 100);
  CHECK(currentMode == MODE_MANUAL && said("ERR:set_alt has no target"), "set_alt refused (it had no target: would drive to 0 mm)");
  stream("0,FORWARD,NONE,manual", 100);
  stream("0,FORWARD,NONE,hold_alt", 300);
  stream("0,FORWARD,both_attach", 30);                 // a dashboard button, through the Pi
  bool held_after_line = currentMode == MODE_HOLD_ALTITUDE;
  stream("0,FORWARD,NONE,hold_alt", delay_to_pose + 100);  // the gamepad keeps streaming, R2 held
  CHECK(held_after_line && currentMode == MODE_HOLD_ALTITUDE, "a 3-field dashboard command does not cancel hold");
  stream("0,FORWARD,NONE,manual", 100);

  std::cout << "grippers\n";
  Serial.out.clear(); stream("0,FORWARD,up_detach,manual", 30);
  CHECK(said("ACK:up_detach") && upperGripperState == GRIPPER_OPEN, "up_detach opens the upper gripper");
  stream("0,FORWARD,both_detach,manual", 30);
  CHECK(said("ERR:busy:both_detach"), "second gripper command during the move is refused");
  idle(delay_to_pose + 100); Serial.out.clear(); stream("0,FORWARD,down_detach,manual", 30);
  CHECK(said("ERR:interlock") && lowerGripperState == GRIPPER_CLOSED, "down_detach refused while the upper is open");
  stream("0,FORWARD,up_attach,manual", 30); idle(delay_to_pose + 100);
  CHECK(upperGripperState == GRIPPER_CLOSED && lowerGripperState == GRIPPER_CLOSED, "up_attach: both recorded closed, as both are");
  Serial.out.clear(); stream("0,FORWARD,axon_open,manual", 30);
  CHECK(said("ACK:axon_open") && bar(chAxon) == 2000, "D-pad left: Axon on 0x40 channel 8 opens");
  Serial.out.clear(); stream("0,FORWARD,axon_close,manual", 30);
  CHECK(said("ACK:axon_close") && bar(chAxon) == 1150, "D-pad right: Axon closes");
  CHECK(boards_kept_apart(), "grippers only on board 0x60 (ch 0-3), bars and Axon only on 0x40");
  Serial.out.clear(); stream("0,FORWARD,wave_hello,manual", 30);
  CHECK(said("ERR:unknown cmd:wave_hello") && !said("ACK:wave_hello"), "unknown command reported, not ACKed");

  std::cout << "pins (ESP32-S3)\n";
  stream("120,FORWARD,NONE,manual", 100);
  CHECK(g_pin[MOTOR_A_IN1] == 1 && g_pin[MOTOR_A_IN2] == 0 && g_pin[MOTOR_B_IN1] == 1 && g_pin[MOTOR_B_IN2] == 0,
        "forward: IN1/IN3 high, IN2/IN4 low");
  {
    bool ok = true;   // S3: GPIO 22-25 don't exist, 26-32 are the flash/PSRAM pins
    for (auto& m : {g_pin, g_pwm}) for (auto& kv : m) if ((kv.first >= 22 && kv.first <= 32) || kv.first > 48) ok = false;
    CHECK(ok, "every pin the firmware drives exists on an ESP32-S3 and is not a flash pin");
  }

  std::cout << "I2C pins that don't exist\n";
  g_i2c_ok = false; Serial.out.clear(); setup();
  CHECK(said("INIT: I2C failed to start on SDA=16 SCL=17"), "boot says so when Wire.begin fails");

  std::cout << (fails ? "FAILED: " + std::to_string(fails) : std::string("ALL PASSED")) << "\n";
  return fails != 0;
}
