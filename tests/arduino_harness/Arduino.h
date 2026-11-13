// Minimal host-side stand-in for the Arduino core, just enough to compile and run the serial-control sketches on a
// desktop for tests. Serial is fed from a scripted byte stream; every motor/servo/serial-out call is logged as an
// event line so the JS port (web/static/serial_engine.js) can be compared with the original C++ (scripts/make_serial_fixture.py).
#pragma once
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

typedef bool boolean;
typedef unsigned char byte;
#define PI 3.14159265358979

void harness_event(const std::string &e);

class String {
 public:
  std::string s;
  String() {}
  String(const char *c) : s(c) {}
  String(const std::string &c) : s(c) {}
  String substring(unsigned a) const { return a > s.size() ? String() : String(s.substr(a)); }
  String substring(unsigned a, unsigned b) const { return a > s.size() ? String() : String(s.substr(a, b - a)); }
  void toCharArray(char *buf, unsigned n) const {  // like Arduino: copies at most n-1 chars and terminates
    if (!n) return;
    size_t k = s.size() < n - 1 ? s.size() : n - 1;
    s.copy(buf, k); buf[k] = 0;
  }
  String &operator+=(char c) { s.push_back(c); return *this; }
  bool operator==(const char *c) const { return s == c; }
  unsigned length() const { return s.size(); }
};

// Byte stream with a gap of `gap` empty polls before each byte after the first (models bytes still arriving).
class HarnessSerial {
 public:
  std::vector<int> data; size_t pos = 0; int gap = 0, wait = 0; long empty_after_end = 0;
  void begin(long) {}
  bool more() const { return pos < data.size(); }
  int available() {
    if (!more()) { if (++empty_after_end > 100000) { harness_event("HANG"); std::exit(3); } return 0; }
    if (wait > 0) { wait--; return 0; }
    return 1;
  }
  int read() {
    if (!more()) { if (++empty_after_end > 100000) { harness_event("HANG"); std::exit(3); } return -1; }
    if (wait > 0) { wait--; return -1; }
    wait = gap;
    return data[pos++];
  }
  void print(const char *t) { harness_event(std::string("TX:") + t); }
  void print(int v) { harness_event("TX:" + std::to_string(v)); }
  void println(const char *t) { harness_event(std::string("TX:") + t + "\\n"); }
  void println(int v) { harness_event("TX:" + std::to_string(v) + "\\n"); }
};
extern HarnessSerial Serial;
inline void delay(unsigned long) {}
