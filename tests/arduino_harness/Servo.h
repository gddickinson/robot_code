// Host-side stand-in for the Arduino Servo library (logs write() with the attached pin).
#pragma once
#include "Arduino.h"
class Servo {
 public:
  int pin = -1;
  void attach(int p) { pin = p; }
  void write(int a) { harness_event("S" + std::to_string(pin) + ".write(" + std::to_string(a) + ")"); }
};
