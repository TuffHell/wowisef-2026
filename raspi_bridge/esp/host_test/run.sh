#!/usr/bin/env bash
# Runs esp_bridge.ino and esp_revamped.ino's command logic on this computer (no ESP32 needed): the
# laptop's message lines go in; servo pulses, drive PWM, modes and replies are checked. Needs Arduino
# IDE 2 (its arduino-cli adds the function declarations the IDE would) and a C++ compiler.
#   bash raspi_bridge/esp/host_test/run.sh
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
CLI="${ARDUINO_CLI:-/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli}"
mkdir -p "$HERE/build"
status=0
for sketch in esp_bridge esp_revamped; do
  echo "=== $sketch"
  "$CLI" compile --fqbn esp32:esp32:esp32s3 --preprocess "$HERE/../$sketch" > "$HERE/build/${sketch}_pre.cpp"
  c++ -std=gnu++17 -Wall -Wno-unused-function -I"$HERE/shim" -I"$HERE" -o "$HERE/build/test_$sketch" "$HERE/test_$sketch.cpp"
  "$HERE/build/test_$sketch" || status=1
done
exit $status
