#pragma once

/* SunSeek Platform — MODULE LAYER
   [Team 2026-10-08] Module_YawEKF.h — relative yaw from gyro + magnetometer (+ accelerometer tilt)

   WHAT IT IS
     A 2-state Extended Kalman Filter for the one axis this spacecraft controls.

       state         x = [ psi , b ]      psi = yaw (heading) [rad], b = gyro bias [rad/s]
       prediction    psi <- psi + (w_gyro - b) dt          every control cycle (50 Hz)
                     b   <- b                              (slow random walk)
       measurement   z = B [ cos a(psi) , sin a(psi) ]     the horizontal magnetic field as the
                     a(psi) = SIGN (psi - OFFSET)          magnetometer sees it in the body frame

     The measurement is a vector that depends on psi through cos/sin: that is the
     non-linear part, and it is linearised around the predicted yaw each cycle
     (H = dz/dpsi = B * tangent). With equal noise on both magnetometer axes only
     the tangential part of the residual carries yaw information, so the 2-D update
     reduces exactly to the scalar one coded below; the radial part (field strength)
     is used as a quality check instead.

     The accelerometer does NOT measure yaw: gravity lies along the rotation axis
     and reads the same at every yaw angle. It measures tilt. Its low-passed
     gravity vector gives the tilt angle (telemetry) and, optionally, the
     horizontal plane onto which the magnetic field is projected before the
     update (tilt compensation, Config_YawEKF.h).

   OUTPUT
     Relative yaw = psi - psi_zero, wrapped to (-180, +180] deg, positive = rotated
     right (same sense as SUN_ANGLE and MAG_HEADING). psi_zero is the zero datum:
     the attitude at the first sample after boot until YAW_ZERO is sent.

   This file has no hardware access and no Arduino calls: Module_ADCS.h feeds it the
   samples it has already read. tools/hosttest_yaw/ compiles it on a PC unchanged. */

#include <math.h>
#include <stdint.h>
#include "Config_Sensor.h"
#include "Config_YawEKF.h"

#define YK_D2R    0.017453292519943295f
#define YK_R2D    57.29577951308232f
#define YK_PI     3.14159265358979f
#define YK_TWO_PI 6.28318530717959f

struct YawEkfInput {
  float dt;                 // [s] since the previous step (0 on the first)
  float rateDps;            // yaw rate, platform sign (+ = turning right), config bias removed
  float swingXDps, swingYDps; // the two other gyro axes (swing / handling detector)
  float magX, magY, magZ;   // calibrated field, any consistent unit
  bool  magNew;             // false = the magnetometer has not produced a new sample
  float accX, accY, accZ;   // accelerometer, raw counts
  bool  accValid;
};

struct YawEkfOut {
  bool  valid;
  float relDeg;       // relative yaw (-180, +180]            <- what the controller uses
  float absDeg;       // estimated heading 0..360 (same sense as MAG_HEADING)
  float magDeg;       // this cycle's magnetometer heading 0..360 (tilt-compensated if enabled)
  float magRelDeg;    // the same, relative to the zero datum
  float rateDps;      // yaw rate with the estimated bias removed
  float biasDps;      // estimated gyro bias left over after GYRO_BIAS_Z_DPS
  float sigmaDeg;     // 1-sigma uncertainty of the yaw estimate, from the covariance
  float innovDeg;     // last magnetometer residual (measured - predicted)
  bool  zeroed;       // false = datum is still the boot attitude
};

struct YawEkfStatus {
  float zeroAbsDeg;       // heading of the zero datum
  float sigmaBiasDps;
  float innovRmsDeg;      // running rms of the residuals that were used
  float gainYaw;          // last Kalman gain on yaw (0..1)
  float fieldRef, field;  // tracked / current horizontal field strength
  float varScale;         // variance multiplier applied to the last sample (1 = clean)
  float tiltDeg;          // tilt of the yaw axis from vertical (accelerometer)
  bool  accOk;
  bool  tiltCompOn, tiltCompAllowed, tiltCompUsed;
  float magSigmaDeg;
  uint32_t used, rejected, reacquired;
  uint32_t execUs, execUsMax;
};

struct YawEkfState {
  bool  init;
  float psi, bias;            // [rad], [rad/s]
  float P00, P01, P11;        // covariance of [psi, bias]
  float zeroPsi; bool datumSet; bool zeroed;
  float prevRate; bool havePrevRate;
  float bRef;                 // tracked horizontal field strength
  float magDt;                // time since the last new magnetometer sample
  float rejectS;              // how long residuals have been outside the gate
  float accLp[3]; bool accInit; uint16_t accN; bool accOk; float up[3]; float tiltRad;
  float swMeanX, swMeanY; bool swInit;
  bool  tiltCompOn;
  float magSigmaRad;
  // diagnostics
  float magPsi, innov, innovMs, gain, field, varScale, rateCorr;
  bool  tiltUsed;
  uint32_t used, rejected, reacquired, execUs, execUsMax;
};

static YawEkfState _yk = {
  false, 0, 0, 0, 0, 0,
  0, false, false,
  0, false,
  0, 0, 0,
  {0, 0, 0}, false, 0, false, {0, 0, 1}, 0,
  0, 0, false,
  (YAW_EKF_TILT_COMP != 0),
  YAW_EKF_MAG_SIGMA_DEG * YK_D2R,
  0, 0, 0, 0, 0, 1, 0,
  false,
  0, 0, 0, 0, 0
};

inline float _ykSq(float x) { return x * x; }
inline float _ykWrapPi(float a)  { while (a > YK_PI) a -= YK_TWO_PI; while (a <= -YK_PI) a += YK_TWO_PI; return a; }
inline float _ykWrap2Pi(float a) { while (a >= YK_TWO_PI) a -= YK_TWO_PI; while (a < 0.0f) a += YK_TWO_PI; return a; }

// Tilt compensation needs a real Z calibration. A planar (turntable) MAG_CAL leaves
// MAG_SCALE_Z far from 1 because Z hardly changed during it.
inline bool yawEkfTiltCompAllowed() { return (MAG_SCALE_Z >= 0.5f) && (MAG_SCALE_Z <= 2.0f); }

inline void yawEkfReset() {           // re-initialise from the next magnetometer sample; keeps the zero datum
  _yk.init = false; _yk.havePrevRate = false; _yk.rejectS = 0; _yk.magDt = 0;
  _yk.accInit = false; _yk.accOk = false; _yk.swInit = false;
}
inline void yawEkfBegin() { yawEkfReset(); _yk.datumSet = false; _yk.zeroed = false; }

inline bool yawEkfValid() { return _yk.init; }

// Zero datum: "the attitude I am in now is 0 deg". Returns the heading of the datum.
inline float yawEkfZero() { _yk.zeroPsi = _yk.psi; _yk.datumSet = true; _yk.zeroed = true; return _yk.psi * YK_R2D; }

inline bool yawEkfSetMagSigma(float deg) {
  if (!isfinite(deg) || deg < 0.5f || deg > 45.0f) return false;
  _yk.magSigmaRad = deg * YK_D2R; return true;
}
inline bool yawEkfSetTiltComp(bool on) {
  if (on && !yawEkfTiltCompAllowed()) return false;
  _yk.tiltCompOn = on; return true;
}
inline void yawEkfNoteExecMicros(uint32_t us) { _yk.execUs = us; if (us > _yk.execUsMax) _yk.execUsMax = us; }

inline bool yawEkfStep(const YawEkfInput &in) {
  if (!isfinite(in.dt) || !isfinite(in.rateDps) || !isfinite(in.magX) || !isfinite(in.magY) || !isfinite(in.magZ) || in.dt < 0.0f) return _yk.init;
  const float dt = in.dt;

  // ---- 1. accelerometer: low-passed gravity direction -> tilt ---------------------------------
  if (in.accValid && isfinite(in.accX) && isfinite(in.accY) && isfinite(in.accZ)) {
    if (!_yk.accInit) { _yk.accLp[0] = in.accX; _yk.accLp[1] = in.accY; _yk.accLp[2] = in.accZ; _yk.accInit = true; _yk.accN = 1; }
    else {
      // plain average over the first samples, then a first-order low-pass
      float k = dt / (YAW_EKF_ACC_TAU_S + dt);
      if (_yk.accN < 1000) { _yk.accN++; const float k0 = 1.0f / (float)_yk.accN; if (k0 > k) k = k0; }
      _yk.accLp[0] += k * (in.accX - _yk.accLp[0]);
      _yk.accLp[1] += k * (in.accY - _yk.accLp[1]);
      _yk.accLp[2] += k * (in.accZ - _yk.accLp[2]);
    }
    const float n = sqrtf(_ykSq(_yk.accLp[0]) + _ykSq(_yk.accLp[1]) + _ykSq(_yk.accLp[2]));
    _yk.accOk = fabsf(n / YAW_EKF_ACC_1G_COUNTS - 1.0f) < YAW_EKF_ACC_NORM_TOL;
    if (_yk.accOk) {
      _yk.up[0] = _yk.accLp[0] / n; _yk.up[1] = _yk.accLp[1] / n; _yk.up[2] = _yk.accLp[2] / n;
      float c = fabsf(_yk.up[2]); if (c > 1.0f) c = 1.0f;
      _yk.tiltRad = acosf(c);
    }
  }

  // ---- 2. swing detector: rate about the two other axes, slow mean (their bias) removed -------
  float swing = 0.0f;
  if (isfinite(in.swingXDps) && isfinite(in.swingYDps)) {
    if (!_yk.swInit) { _yk.swMeanX = in.swingXDps; _yk.swMeanY = in.swingYDps; _yk.swInit = true; }
    else { const float k = dt / (5.0f + dt); _yk.swMeanX += k * (in.swingXDps - _yk.swMeanX); _yk.swMeanY += k * (in.swingYDps - _yk.swMeanY); }
    swing = sqrtf(_ykSq(in.swingXDps - _yk.swMeanX) + _ykSq(in.swingYDps - _yk.swMeanY));
  }

  // ---- 3. measurement: horizontal field components -------------------------------------------
  float zx = in.magX, zy = in.magY;
  _yk.tiltUsed = false;
  if (_yk.tiltCompOn && _yk.accOk && _yk.accN >= 25 && yawEkfTiltCompAllowed()) {   // 25 samples: gravity averaged for 0.5 s
    const float ux = _yk.up[0], uy = _yk.up[1], uz = _yk.up[2];
    // e1 = body X projected onto the horizontal plane, e2 completes it so that level -> (X, Y)
    float e1x = 1.0f - ux * ux, e1y = -ux * uy, e1z = -ux * uz;
    const float n1 = sqrtf(e1x * e1x + e1y * e1y + e1z * e1z);
    if (n1 > 0.5f && fabsf(uz) > 0.5f) {
      e1x /= n1; e1y /= n1; e1z /= n1;
      const float s = (uz >= 0.0f) ? 1.0f : -1.0f;
      const float e2x = s * (uy * e1z - uz * e1y), e2y = s * (uz * e1x - ux * e1z), e2z = s * (ux * e1y - uy * e1x);
      zx = in.magX * e1x + in.magY * e1y + in.magZ * e1z;
      zy = in.magX * e2x + in.magY * e2y + in.magZ * e2z;
      _yk.tiltUsed = true;
    }
  }
  const float zn = sqrtf(zx * zx + zy * zy);
  const bool magUsable = zn > 1.0e-6f;
  const float offRad = MAG_HEADING_OFFSET_DEG * YK_D2R;
  if (magUsable) { _yk.magPsi = _ykWrap2Pi(MAG_HEADING_SIGN * atan2f(zy, zx) + offRad); _yk.field = zn; }

  // ---- 4. first sample: start on the magnetometer heading -------------------------------------
  if (!_yk.init) {
    if (!magUsable) return false;
    _yk.psi = _yk.magPsi; _yk.bias = 0.0f;
    _yk.P00 = _ykSq(_yk.magSigmaRad); _yk.P01 = 0.0f; _yk.P11 = _ykSq(YAW_EKF_INIT_BIAS_SIGMA * YK_D2R);
    _yk.bRef = zn; _yk.magDt = 0.0f; _yk.rejectS = 0.0f;
    _yk.prevRate = in.rateDps * YK_D2R; _yk.havePrevRate = true;
    _yk.innov = 0.0f; _yk.gain = 1.0f; _yk.varScale = 1.0f; _yk.rateCorr = in.rateDps * YK_D2R;
    if (!_yk.datumSet) { _yk.zeroPsi = _yk.psi; _yk.datumSet = true; _yk.zeroed = false; }
    _yk.init = true;
    return true;
  }

  // ---- 5. prediction: integrate the gyro ------------------------------------------------------
  const float w = in.rateDps * YK_D2R;
  if (dt > YAW_EKF_MAX_DT_S) {
    // not sampled for too long: do not integrate, admit the yaw may have changed
    _yk.P00 += _ykSq(60.0f * YK_D2R * dt);
    _yk.rateCorr = w - _yk.bias;
  } else if (dt > 0.0f) {
    const float wAvg = _yk.havePrevRate ? 0.5f * (w + _yk.prevRate) : w;   // trapezoid between samples
    const float wc = wAvg - _yk.bias;
    _yk.psi = _ykWrap2Pi(_yk.psi + wc * dt);
    // P <- F P F' + Q,  F = [1 -dt; 0 1]
    _yk.P00 += dt * (-2.0f * _yk.P01 + dt * _yk.P11)
             + _ykSq(YAW_EKF_GYRO_ARW * YK_D2R) * dt
             + _ykSq(YAW_EKF_GYRO_SCALE_UNC * wc) * dt;        // scale-factor error, ~1 s correlation
    _yk.P01 -= dt * _yk.P11;
    _yk.P11 += _ykSq(YAW_EKF_BIAS_RW * YK_D2R) * dt;
    _yk.rateCorr = w - _yk.bias;
  }
  if (_yk.P00 > _ykSq(YK_PI)) _yk.P00 = _ykSq(YK_PI);
  _yk.prevRate = w; _yk.havePrevRate = true;
  _yk.magDt += dt;
  if (_yk.rejectS > 0.0f) _yk.rejectS += dt;

  // ---- 6. correction: magnetometer -------------------------------------------------------------
  if (in.magNew && magUsable) {
    // field-strength tracking (slow) and anomaly of this sample
    const float kB = _yk.magDt / (YAW_EKF_FIELD_TAU_S + _yk.magDt);
    const float anomaly = zn / _yk.bRef - 1.0f;
    _yk.bRef += kB * (zn - _yk.bRef);
    _yk.magDt = 0.0f;

    // predicted field direction in the sensor frame, and the tangent (direction of +yaw)
    const float a = MAG_HEADING_SIGN * (_yk.psi - offRad);
    const float ca = cosf(a), sa = sinf(a);
    const float tx = -MAG_HEADING_SIGN * sa, ty = MAG_HEADING_SIGN * ca;
    const float e = (zx * tx + zy * ty) / _yk.bRef;          // EKF residual, in rad for small errors
    const float delta = _ykWrapPi(_yk.magPsi - _yk.psi);     // the full angular residual (for the gate)

    // measurement variance: nominal, inflated when the sample looks disturbed
    const float k2 = 1.0f + _ykSq(anomaly / YAW_EKF_FIELD_TOL) + _ykSq(swing / YAW_EKF_SWING_RATE_DPS);
    const float R = _ykSq(_yk.magSigmaRad) * k2;
    const float S = _yk.P00 + R;
    _yk.varScale = k2;
    _yk.innov = delta;

    if (_ykSq(delta) > _ykSq(YAW_EKF_GATE_SIGMA) * S) {
      // outside the gate: an outlier, or the estimate itself is wrong
      _yk.rejected++;
      if (_yk.rejectS <= 0.0f) _yk.rejectS = 1.0e-6f;         // start the clock
      _yk.gain = 0.0f;
      if (_yk.rejectS >= YAW_EKF_REACQUIRE_S) {
        _yk.psi = _yk.magPsi; _yk.P00 = R; _yk.P01 = 0.0f;
        const float p11 = _ykSq(0.5f * YAW_EKF_INIT_BIAS_SIGMA * YK_D2R);
        if (_yk.P11 < p11) _yk.P11 = p11;
        _yk.rejectS = 0.0f; _yk.reacquired++;
      }
    } else {
      const float K0 = _yk.P00 / S;
      const float K1 = (fabsf(_yk.rateCorr) <= YAW_EKF_BIAS_LEARN_MAX_DPS * YK_D2R) ? _yk.P01 / S : 0.0f;
      _yk.psi = _ykWrap2Pi(_yk.psi + K0 * e);
      _yk.bias += K1 * e;
      const float bMax = YAW_EKF_BIAS_MAX_DPS * YK_D2R;
      if (_yk.bias > bMax) _yk.bias = bMax; else if (_yk.bias < -bMax) _yk.bias = -bMax;
      // Joseph form, valid for any gain (K1 may have been forced to 0)
      const float P00 = _yk.P00, P01 = _yk.P01, P11 = _yk.P11;
      _yk.P00 = _ykSq(1.0f - K0) * P00 + K0 * K0 * R;
      _yk.P01 = (1.0f - K0) * (P01 - K1 * P00) + K0 * K1 * R;
      _yk.P11 = P11 - 2.0f * K1 * P01 + K1 * K1 * (P00 + R);
      _yk.rateCorr = w - _yk.bias;
      _yk.gain = K0; _yk.rejectS = 0.0f; _yk.used++;
      _yk.innovMs += 0.01f * (_ykSq(delta) - _yk.innovMs);     // ~2 s window at 50 Hz
    }
  }
  if (!isfinite(_yk.psi) || !isfinite(_yk.bias) || !isfinite(_yk.P00) || !isfinite(_yk.P01) || !isfinite(_yk.P11)) { yawEkfReset(); return false; }
  return true;
}

inline float _ykWrap180Deg(float d) { while (d > 180.0f) d -= 360.0f; while (d <= -180.0f) d += 360.0f; return d; }

inline YawEkfOut yawEkfGet() {
  YawEkfOut o;
  o.valid = _yk.init;
  o.absDeg = _yk.psi * YK_R2D;
  o.relDeg = _ykWrap180Deg((_yk.psi - _yk.zeroPsi) * YK_R2D);
  o.magDeg = _yk.magPsi * YK_R2D;
  o.magRelDeg = _ykWrap180Deg((_yk.magPsi - _yk.zeroPsi) * YK_R2D);
  o.rateDps = _yk.rateCorr * YK_R2D;
  o.biasDps = _yk.bias * YK_R2D;
  o.sigmaDeg = sqrtf(_yk.P00 > 0.0f ? _yk.P00 : 0.0f) * YK_R2D;
  o.innovDeg = _yk.innov * YK_R2D;
  o.zeroed = _yk.zeroed;
  return o;
}

inline YawEkfStatus yawEkfStatus() {
  YawEkfStatus s;
  s.zeroAbsDeg = _yk.zeroPsi * YK_R2D;
  s.sigmaBiasDps = sqrtf(_yk.P11 > 0.0f ? _yk.P11 : 0.0f) * YK_R2D;
  s.innovRmsDeg = sqrtf(_yk.innovMs > 0.0f ? _yk.innovMs : 0.0f) * YK_R2D;
  s.gainYaw = _yk.gain;
  s.fieldRef = _yk.bRef; s.field = _yk.field;
  s.varScale = _yk.varScale;
  s.tiltDeg = _yk.tiltRad * YK_R2D;
  s.accOk = _yk.accOk;
  s.tiltCompOn = _yk.tiltCompOn; s.tiltCompAllowed = yawEkfTiltCompAllowed(); s.tiltCompUsed = _yk.tiltUsed;
  s.magSigmaDeg = _yk.magSigmaRad * YK_R2D;
  s.used = _yk.used; s.rejected = _yk.rejected; s.reacquired = _yk.reacquired;
  s.execUs = _yk.execUs; s.execUsMax = _yk.execUsMax;
  return s;
}
