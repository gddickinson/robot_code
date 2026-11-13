// Host-side stand-in for the Adafruit Motor Shield v2 library (logs run()/setSpeed() calls).
#pragma once
#include "Arduino.h"
enum { FORWARD = 1, BACKWARD = 2, BRAKE = 3, RELEASE = 4 };
class Adafruit_DCMotor {
 public:
  int n = 0;
  void run(int dir) {
    static const char *names[] = {"?", "FORWARD", "BACKWARD", "BRAKE", "RELEASE"};
    harness_event("AF" + std::to_string(n) + ".run(" + names[dir] + ")");
  }
  void setSpeed(int s) { harness_event("AF" + std::to_string(n) + ".setSpeed(" + std::to_string(s) + ")"); }
};
class Adafruit_MotorShield {
 public:
  Adafruit_DCMotor m[5];
  void begin(int = 1600) {}
  Adafruit_DCMotor *getMotor(int k) { m[k].n = k; return &m[k]; }
};
