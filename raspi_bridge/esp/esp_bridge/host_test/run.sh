#!/usr/bin/env bash
# Runs esp_bridge.ino's command logic on this computer (no ESP32 needed): the laptop's message
# lines go in, servo pulses, drive PWM and replies are checked. Needs Arduino IDE 2 (for its
# arduino-cli, which adds the function declarations the IDE would) and a C++ compiler.
#   bash raspi_bridge/esp/esp_bridge/host_test/run.sh
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
CLI="${ARDUINO_CLI:-/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli}"
mkdir -p "$HERE/build"
"$CLI" compile --fqbn esp32:esp32:esp32 --preprocess "$HERE/.." > "$HERE/build/esp_bridge_pre.cpp"
c++ -std=gnu++17 -Wall -I"$HERE/shim" -I"$HERE" -o "$HERE/build/test_esp_bridge" "$HERE/test_esp_bridge.cpp"
"$HERE/build/test_esp_bridge"
