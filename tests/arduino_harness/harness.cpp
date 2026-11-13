// Runs one serial-control sketch on the desktop: reads `gap` and the byte stream from argv, calls setup() and then
// loop() until every byte has been consumed (plus one more loop), printing one event per line.
// Build: c++ -std=c++11 -I tests/arduino_harness -include tests/arduino_harness/prototypes.h \
//          -DSKETCH='"path/to/sketch.ino"' tests/arduino_harness/harness.cpp
#include <iostream>
#include "Arduino.h"

HarnessSerial Serial;
void harness_event(const std::string &e) { std::cout << e << "\n"; }

#include SKETCH

int main(int argc, char **argv) {
  if (argc < 3) { std::cerr << "usage: harness GAP STREAM\n"; return 2; }
  Serial.gap = std::atoi(argv[1]);
  for (const char *p = argv[2]; *p; ++p) Serial.data.push_back((unsigned char)*p);
  setup();
  std::cout << "--- loop\n";
  for (int i = 0; i < 100000 && Serial.more(); i++) { loop(); std::cout << "--- loop\n"; }
  loop();
  return 0;
}
