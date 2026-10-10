#pragma once

/* SunSeek Platform v2.0 — MODULE LAYER */

#include <Arduino.h>
#include <math.h>
#include "Config_Estimator.h"

enum EstimatorReference { EST_REF_SUN = 0, EST_REF_MAG = 1 };
enum EstimatorFilterType { EST_FILTER_CUSTOM = 0, EST_FILTER_MOVING_AVERAGE = 1, EST_FILTER_LPF = 2 };

struct EstimatorState {
  bool filterEnabled;
  EstimatorFilterType filterType;
  uint16_t movingAverageWindow;
  float filterStrength;   // LPF: 0 Fast ... 1 Smooth
  bool fusionEnabled;
  float gyroWeight;       // 0 Reference ... 1 Gyro

  float rawReferenceDeg;
  float filteredReferenceDeg;
  float estimatedAngleDeg;
  float gyroRateDps;
  float dtSec;
  bool valid;
};

static EstimatorState _est = {
  ESTIMATOR_DEFAULT_FILTER_ENABLED,
  (EstimatorFilterType)ESTIMATOR_DEFAULT_FILTER_TYPE,
  ESTIMATOR_DEFAULT_MA_WINDOW,
  ESTIMATOR_DEFAULT_FILTER_STRENGTH,
  ESTIMATOR_DEFAULT_FUSION_ENABLED,
  ESTIMATOR_DEFAULT_GYRO_WEIGHT,
  0, 0, 0, 0, 0, false
};

static EstimatorReference _estRef = EST_REF_SUN;
static unsigned long _estLastUs = 0;
static float _estMa[ESTIMATOR_MAX_MA_WINDOW];
static uint16_t _estMaCount = 0;
static uint16_t _estMaHead = 0;
static bool _estExternal = false;   // [Team] true while the yaw EKF supplies the estimate (ADCS_REFERENCE,YAW)

inline float estimatorWrap180(float x) { while (x > 180.0f) x -= 360.0f; while (x <= -180.0f) x += 360.0f; return x; }
inline float estimatorWrap360(float x) { while (x >= 360.0f) x -= 360.0f; while (x < 0.0f) x += 360.0f; return x; }
inline float estimatorAngleDiff(float target, float current) { return estimatorWrap180(target-current); }
inline float estimatorNormalize(float x, EstimatorReference ref) { return ref==EST_REF_MAG ? estimatorWrap360(x) : x; }

inline const char* estimatorFilterTypeText() {
  if (_estExternal) return "EKF_YAW";   // [Team]
  if (_est.filterType==EST_FILTER_MOVING_AVERAGE) return "MOVING_AVERAGE";
  if (_est.filterType==EST_FILTER_LPF) return "LPF";
  return "CUSTOM";
}
inline void estimatorReset() { _est.valid=false; _estLastUs=0; _estMaCount=0; _estMaHead=0; _estExternal=false; }
// [Team] ADCS_REFERENCE,YAW: the filter + fusion chain below is bypassed and Module_YawEKF.h
// supplies the estimate. Storing it here keeps EST_RAW / EST_FILTERED / EST_ANGLE telemetry alive.
inline void estimatorSetExternal(float rawDeg, float filteredDeg, float estimatedDeg, float rateDps, float dtSec) {
  _est.rawReferenceDeg=rawDeg; _est.filteredReferenceDeg=filteredDeg; _est.estimatedAngleDeg=estimatedDeg;
  _est.gyroRateDps=rateDps; _est.dtSec=dtSec; _est.valid=true;
  _estExternal=true; _estLastUs=0; _estMaCount=0; _estMaHead=0;
}
inline void estimatorBegin() { estimatorReset(); }
inline EstimatorState estimatorGet() { return _est; }

inline bool estimatorSetFilterEnabled(bool on) { _est.filterEnabled=on; estimatorReset(); return true; }
// Backward-compatible LPF ON/OFF command now means master filter enable/disable.
inline bool estimatorSetLPFEnabled(bool on) { return estimatorSetFilterEnabled(on); }

inline bool estimatorSetFilterType(EstimatorFilterType type) {
  if (type!=EST_FILTER_MOVING_AVERAGE && type!=EST_FILTER_LPF && type!=EST_FILTER_CUSTOM) return false;
  _est.filterType=type; estimatorReset(); return true;
}
inline bool estimatorSetMovingAverageWindow(int n) {
  if (n<1 || n>ESTIMATOR_MAX_MA_WINDOW) return false;
  _est.movingAverageWindow=(uint16_t)n; estimatorReset(); return true;
}
inline bool estimatorSetFilterStrength(float strength) {
  if (!isfinite(strength) || strength<0.0f || strength>1.0f) return false;
  _est.filterStrength=strength; estimatorReset(); return true;
}
inline bool estimatorSetFusionEnabled(bool on) { _est.fusionEnabled=on; estimatorReset(); return true; }
inline bool estimatorSetGyroWeight(float weight) {
  if (!isfinite(weight) || weight<0.0f || weight>1.0f) return false;
  _est.gyroWeight=weight; estimatorReset(); return true;
}
inline void estimatorSetReference(EstimatorReference ref) { if (_estRef!=ref) { _estRef=ref; estimatorReset(); } }

inline float estimatorMovingAverage(float sample, EstimatorReference ref) {
  uint16_t n=_est.movingAverageWindow;
  _estMa[_estMaHead]=sample; _estMaHead=(_estMaHead+1)%n; if (_estMaCount<n) _estMaCount++;
  if (ref==EST_REF_MAG) {
    float ss=0.0f, cc=0.0f;
    for (uint16_t i=0;i<_estMaCount;i++) { float r=_estMa[i]*DEG_TO_RAD; ss+=sinf(r); cc+=cosf(r); }
    return estimatorWrap360(atan2f(ss,cc)*RAD_TO_DEG);
  }
  float sum=0.0f; for (uint16_t i=0;i<_estMaCount;i++) sum+=_estMa[i];
  return sum/(float)_estMaCount;
}

inline bool estimatorUpdate(float rawReferenceDeg, float gyroRateDps, EstimatorReference ref) {
  if (!isfinite(rawReferenceDeg) || !isfinite(gyroRateDps)) { _est.valid=false; return false; }
  if (_estExternal) estimatorReset();   // [Team] coming back from the yaw EKF: start the chain clean
  estimatorSetReference(ref);
  unsigned long nowUs=micros(); float dt=0.0f;
  if (_estLastUs!=0) dt=(float)(nowUs-_estLastUs)*1.0e-6f; _estLastUs=nowUs;
  _est.rawReferenceDeg=estimatorNormalize(rawReferenceDeg,ref); _est.gyroRateDps=gyroRateDps; _est.dtSec=dt;

  if (!_est.valid || dt<ESTIMATOR_MIN_DT_S || dt>ESTIMATOR_MAX_DT_S) {
    _est.filteredReferenceDeg=_est.rawReferenceDeg; _est.estimatedAngleDeg=_est.rawReferenceDeg;
    _estMaCount=0; _estMaHead=0;
    if (_est.filterEnabled && _est.filterType==EST_FILTER_MOVING_AVERAGE) _est.filteredReferenceDeg=estimatorMovingAverage(_est.rawReferenceDeg,ref);
    _est.valid=true; return true;
  }

  if (!_est.filterEnabled) {
    _est.filteredReferenceDeg=_est.rawReferenceDeg;
  } else if (_est.filterType==EST_FILTER_MOVING_AVERAGE) {
    _est.filteredReferenceDeg=estimatorMovingAverage(_est.rawReferenceDeg,ref);
  } else if (_est.filterType==EST_FILTER_LPF) {
    float alpha=constrain(1.0f-_est.filterStrength,0.0f,1.0f);
    if (ref==EST_REF_MAG) _est.filteredReferenceDeg=estimatorWrap360(_est.filteredReferenceDeg + alpha*estimatorAngleDiff(_est.rawReferenceDeg,_est.filteredReferenceDeg));
    else _est.filteredReferenceDeg += alpha*(_est.rawReferenceDeg-_est.filteredReferenceDeg);
  } else {
    // CUSTOM hook baseline: pass-through until a team custom filter is supplied.
    _est.filteredReferenceDeg=_est.rawReferenceDeg;
  }

  if (_est.fusionEnabled) {
    float predicted=estimatorNormalize(_est.estimatedAngleDeg+gyroRateDps*dt,ref);
    float correction=estimatorAngleDiff(_est.filteredReferenceDeg,predicted);
    _est.estimatedAngleDeg=estimatorNormalize(predicted+(1.0f-_est.gyroWeight)*correction,ref);
  } else _est.estimatedAngleDeg=_est.filteredReferenceDeg;
  return true;
}
