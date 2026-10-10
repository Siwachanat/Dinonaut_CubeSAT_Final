// Minimal Arduino stand-in so src/Module_YawEKF.h and src/Module_Estimator.h compile on a PC unchanged.
#pragma once
#include <stdint.h>
#include <math.h>
#include <cmath>
using std::isfinite;
#ifndef PI
#define PI 3.14159265358979323846f
#endif
#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.29577951308232f
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
extern unsigned long g_fakeMicros;                  // the harness advances this by dt before each estimator call
inline unsigned long micros() { return g_fakeMicros; }
