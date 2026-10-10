// Minimal Arduino stand-in so src/Module_SunSensor.h compiles on a PC unchanged.
#pragma once
#include <stdint.h>
#include <math.h>
#define INPUT 0
extern float g_fakeL, g_fakeR;          // counts the fake ADC returns
extern unsigned long g_fakeMicros;
inline void pinMode(int, int) {}
inline unsigned long micros() { g_fakeMicros += 20000UL; return g_fakeMicros; }   // 20 ms per call: cache never hits
int analogRead(int pin);
inline uint32_t analogReadMilliVolts(int pin) { return (uint32_t)(analogRead(pin) * 3100L / 4095L); }
