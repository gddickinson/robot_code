// Host-side stand-in for the Makeblock library's MeDCMotor (logs calls; the real run() is in libraries/makeblock).
#pragma once
#include "Arduino.h"
#define M1 1
#define M2 2
class MeDCMotor {
 public:
  int port;
  explicit MeDCMotor(int p) : port(p) {}
  void run(int speed) { harness_event("M" + std::to_string(port) + ".run(" + std::to_string(speed) + ")"); }
  void stop() { harness_event("M" + std::to_string(port) + ".stop()"); }
};
