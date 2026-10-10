// Runs the real firmware estimators on a PC, sample by sample.
//   A  src/Module_YawEKF.h      the team's 2-state EKF
//   B  src/Module_Estimator.h   the organisers' chain (moving average + complementary gyro fusion)
// stdin : "dt rate swingX swingY magX magY magZ magNew accX accY accZ"   one line per control cycle
//         or a command: reset | zero | sigma <deg> | tilt <0|1> | fusion <0|1> | ma <n>
// stdout: one line per sample (see printf below)
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "Arduino.h"
unsigned long g_fakeMicros = 1000000UL;
#include "Config_Sensor.h"
#ifdef YAWSIM_SIGN                 // run.py builds the module for both heading signs
#undef MAG_HEADING_SIGN
#define MAG_HEADING_SIGN YAWSIM_SIGN
#endif
#ifdef YAWSIM_OFFSET_DEG
#undef MAG_HEADING_OFFSET_DEG
#define MAG_HEADING_OFFSET_DEG YAWSIM_OFFSET_DEG
#endif
#ifdef YAWSIM_SCALE_Z              // tilt compensation needs a valid Z calibration
#undef MAG_SCALE_Z
#define MAG_SCALE_Z YAWSIM_SCALE_Z
#endif
#include "Module_YawEKF.h"
#include "Module_Estimator.h"

int main() {
  char line[256];
  yawEkfBegin();
  estimatorBegin();
  while (fgets(line, sizeof line, stdin)) {
    if (!strncmp(line, "reset", 5)) { yawEkfReset(); estimatorReset(); continue; }
    if (!strncmp(line, "zero", 4))  { yawEkfZero(); continue; }
    if (!strncmp(line, "sigma", 5)) { printf("# sigma %d\n", yawEkfSetMagSigma((float)atof(line + 5)) ? 1 : 0); continue; }
    if (!strncmp(line, "tilt", 4))  { printf("# tilt %d\n", yawEkfSetTiltComp(atoi(line + 4) != 0) ? 1 : 0); continue; }
    if (!strncmp(line, "fusion", 6)) { estimatorSetFusionEnabled(atoi(line + 6) != 0); continue; }
    if (!strncmp(line, "ma", 2))    { estimatorSetMovingAverageWindow(atoi(line + 2)); continue; }
    YawEkfInput in; int mnew = 1;
    if (sscanf(line, "%f %f %f %f %f %f %f %d %f %f %f", &in.dt, &in.rateDps, &in.swingXDps, &in.swingYDps,
               &in.magX, &in.magY, &in.magZ, &mnew, &in.accX, &in.accY, &in.accZ) != 11) continue;
    in.magNew = mnew != 0; in.accValid = true;
    bool ok = yawEkfStep(in);
    YawEkfOut o = yawEkfGet(); YawEkfStatus s = yawEkfStatus();

    // organisers' path, exactly as imuProcess() + adcsRead() feed it in MAG reference
    float heading = atan2f(in.magY, in.magX) * 180.0f / PI;
    heading = MAG_HEADING_SIGN * heading + MAG_HEADING_OFFSET_DEG;
    while (heading < 0.0f) heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;
    g_fakeMicros += (unsigned long)lroundf(in.dt * 1.0e6f);
    estimatorUpdate(heading, in.rateDps, EST_REF_MAG);
    EstimatorState es = estimatorGet();

    printf("%d %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.3f %lu %lu %lu %.4f %.4f %.4f %.3f %d %.4f\n",
           ok ? 1 : 0, o.absDeg, o.relDeg, o.magDeg, o.biasDps, o.sigmaDeg, o.innovDeg, s.gainYaw, s.varScale, s.tiltDeg,
           (unsigned long)s.used, (unsigned long)s.rejected, (unsigned long)s.reacquired,
           es.estimatedAngleDeg, es.filteredReferenceDeg, heading, o.rateDps, s.tiltCompUsed ? 1 : 0, s.sigmaBiasDps);
  }
  return 0;
}
