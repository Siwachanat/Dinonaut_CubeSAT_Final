// PC stand-in for the Arduino-ESP32 core: just enough to compile and run the WHOLE firmware
// (src/main.cpp and every module) against the simulated spacecraft in ../fwsim.cpp.
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <cmath>
#include <string>
#include <algorithm>
using std::isfinite; using std::min; using std::max;

#ifndef PI
#define PI 3.14159265358979323846f
#endif
#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.29577951308232f
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define HEX 16
#define SERIAL_8N1 0
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
inline long map(long x, long a, long b, long c, long d) { return (x - a) * (d - c) / (b - a) + c; }

class String {
 public:
  std::string s;
  String() {}
  String(const char *c) : s(c ? c : "") {}
  String(const std::string &c) : s(c) {}
  String(char c) : s(1, c) {}
  String(int v) : s(std::to_string(v)) {}
  String(unsigned int v) : s(std::to_string(v)) {}
  String(long v) : s(std::to_string(v)) {}
  String(unsigned long v) : s(std::to_string(v)) {}
  String(unsigned char v, int base) { char b[16]; snprintf(b, sizeof b, base == 16 ? "%x" : "%u", (unsigned)v); s = b; }
  String(float v, int dec = 2) { char b[48]; snprintf(b, sizeof b, "%.*f", dec, (double)v); s = b; }
  String(double v, int dec = 2) { char b[48]; snprintf(b, sizeof b, "%.*f", dec, v); s = b; }
  unsigned int length() const { return (unsigned int)s.size(); }
  const char *c_str() const { return s.c_str(); }
  int indexOf(char c, unsigned int from = 0) const { size_t p = s.find(c, from); return p == std::string::npos ? -1 : (int)p; }
  int indexOf(const char *c, unsigned int from = 0) const { size_t p = s.find(c, from); return p == std::string::npos ? -1 : (int)p; }
  void reserve(unsigned int n) { s.reserve(n); }
  String substring(unsigned int a) const { return a >= s.size() ? String() : String(s.substr(a)); }
  String substring(unsigned int a, unsigned int b) const { if (b > s.size()) b = (unsigned int)s.size(); return a >= b ? String() : String(s.substr(a, b - a)); }
  void trim() { size_t a = s.find_first_not_of(" \t\r\n"); size_t b = s.find_last_not_of(" \t\r\n"); s = (a == std::string::npos) ? "" : s.substr(a, b - a + 1); }
  bool startsWith(const String &p) const { return s.compare(0, p.s.size(), p.s) == 0; }
  bool endsWith(const String &p) const { return s.size() >= p.s.size() && s.compare(s.size() - p.s.size(), p.s.size(), p.s) == 0; }
  void remove(unsigned int i, unsigned int n) { s.erase(i, n); }
  long toInt() const { return atol(s.c_str()); }
  char operator[](unsigned int i) const { return s[i]; }
  String &operator+=(const String &o) { s += o.s; return *this; }
  String &operator+=(const char *o) { s += o; return *this; }
  String &operator+=(char c) { s += c; return *this; }
  bool operator==(const String &o) const { return s == o.s; }
  bool operator==(const char *o) const { return s == o; }
  bool operator!=(const String &o) const { return s != o.s; }
  bool operator!=(const char *o) const { return s != o; }
};
inline String operator+(const String &a, const String &b) { return String(a.s + b.s); }
inline String operator+(const String &a, const char *b) { return String(a.s + b); }
inline String operator+(const char *a, const String &b) { return String(std::string(a) + b.s); }

// ---- provided by ../fwsim.cpp ----
unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void pinMode(int pin, int mode);
void digitalWrite(int pin, int level);
void analogWrite(int pin, int value);
int analogRead(int pin);
uint32_t analogReadMilliVolts(int pin);
void simSerialOut(const char *line);
int simSerialAvailable();
int simSerialRead();
void simPayloadTx(const char *text);     // UART1: what the firmware sends to the ESP32-CAM
int simPayloadAvailable();
int simPayloadRead();

class HardwareSerial {
 public:
  int port;
  HardwareSerial(int p = 0) : port(p) {}
  void begin(unsigned long, ...) {}
  void println(const String &t) { if (port == 0) simSerialOut(t.c_str()); }
  void println(const char *t) { if (port == 0) simSerialOut(t); }
  void print(const String &t) { if (port == 1) simPayloadTx(t.c_str()); }
  void print(const char *t) { if (port == 1) simPayloadTx(t); }
  void print(char c) { if (port == 1) { char b[2] = {c, 0}; simPayloadTx(b); } }
  int available() { return port == 0 ? simSerialAvailable() : simPayloadAvailable(); }
  int read() { return port == 0 ? simSerialRead() : simPayloadRead(); }
};
extern HardwareSerial Serial;
