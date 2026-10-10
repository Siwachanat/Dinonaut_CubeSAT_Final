// I2C bus stand-in: the GY-89 register maps live in ../fwsim.cpp
#pragma once
#include <stdint.h>
class TwoWire {
 public:
  void begin(int, int) {}
  void setClock(unsigned long) {}
  void beginTransmission(uint8_t addr);
  size_t write(uint8_t b);
  uint8_t endTransmission(bool stop = true);
  uint8_t requestFrom(uint8_t addr, uint8_t n);
  int read();
};
extern TwoWire Wire;
