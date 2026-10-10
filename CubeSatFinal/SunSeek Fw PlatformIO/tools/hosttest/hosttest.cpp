// Runs the real firmware estimator (src/Module_SunSensor.h) on a PC.
// stdin : lines "L R" (counts), or "reset" to clear the brightness reference
// stdout: "angle branch X lnSum ampRef" per line, via the same sunSensorRead() the board calls
#include <stdio.h>
#include <string.h>
#include "Arduino.h"
#include "Config_Sensor.h"
#ifdef SUN_ALGORITHM_OVERRIDE      // run.py builds the module once per algorithm
#undef SUN_ALGORITHM
#define SUN_ALGORITHM SUN_ALGORITHM_OVERRIDE
#endif
float g_fakeL = 0, g_fakeR = 0;
unsigned long g_fakeMicros = 0;
int analogRead(int pin) { return (int)lroundf(pin == SUN_LEFT_PIN ? g_fakeL : g_fakeR); }
#include "Module_SunSensor.h"

int main() {
  char line[128];
  while (fgets(line, sizeof line, stdin)) {
    if (!strncmp(line, "reset", 5)) { sunRefReset(); continue; }
    float L, R;
    if (sscanf(line, "%f %f", &L, &R) != 2) continue;
    g_fakeL = L; g_fakeR = R;
    SunSample s;
    bool ok = sunSensorRead(s);
    printf("%.4f %s %.5f %.5f %.5f %d %d %d\n", s.angleDeg, sunBranchText(s.branch), s.x, s.lnSum,
           sunAmpRef(), ok ? 1 : 0, s.leftRaw, s.rightRaw);
  }
  return 0;
}
