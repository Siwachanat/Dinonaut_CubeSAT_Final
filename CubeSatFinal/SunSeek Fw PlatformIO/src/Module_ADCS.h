#pragma once

/* SunSeek Platform v2.0 — MODULE LAYER */
#include <Arduino.h>
#include <math.h>
#include "Config_ADCS.h"
#include "Module_IMU.h"
#include "Module_SunSensor.h"
#include "Module_ReactionWheel.h"
#include "Module_Estimator.h"
#include "Module_YawEKF.h"      // [Team] relative-yaw EKF (gyro + magnetometer + accelerometer tilt)

enum ADCSMode { ADCS_MANUAL, ADCS_AUTO };
enum ADCSReference { ADCS_SUN, ADCS_MAG, ADCS_YAW };   // [Team] ADCS_YAW = relative yaw from Module_YawEKF.h

struct ADCSState {
  ADCSMode mode;
  ADCSReference ref;
  float target, kp, kd;
  int bias;

  // Raw sensor-domain errors retained for Engineering telemetry.
  float sunError, magError;

  // Controller-domain values.
  float error, rate, u;

  // Estimation chain.
  float rawReference;
  float filteredReference;
  float estimatedAngle;

  bool valid;
};

static ADCSState _a = {
  ADCS_MANUAL, ADCS_SUN,
  ADCS_DEFAULT_TARGET_DEG, ADCS_DEFAULT_KP, ADCS_DEFAULT_KD,
  RW_DEFAULT_BIAS_PERCENT,
  0, 0, 0, 0, 0,
  0, 0, 0,
  false
};

static unsigned long _aLast = 0;

inline float adcsWrap180(float x){ while(x>180)x-=360; while(x<=-180)x+=360; return x; }
inline const char* adcsModeText(){return _a.mode==ADCS_AUTO?"AUTO":"MANUAL";}
inline const char* adcsRefText(){return _a.ref==ADCS_YAW?"YAW":(_a.ref==ADCS_MAG?"MAG":"SUN");}

inline void adcsBegin(){
  estimatorBegin();
  yawEkfBegin();
  _aLast=millis();
}

inline ADCSState adcsGet(){return _a;}
inline void adcsManual(){_a.mode=ADCS_MANUAL;_a.u=0;}

inline bool adcsReference(ADCSReference r){
  if(_a.mode==ADCS_AUTO)return false;
  if(r!=_a.ref) estimatorReset();   // [Team] always restart the standard chain on a reference change
  _a.ref=r;
  if(r!=ADCS_YAW) estimatorSetReference(r==ADCS_MAG ? EST_REF_MAG : EST_REF_SUN);
  return true;
}

inline bool adcsTarget(float t){
  if(_a.mode==ADCS_AUTO)return false;
  if(_a.ref==ADCS_SUN&&(t < -90 || t > 90))return false;
  if(_a.ref==ADCS_MAG&&(t<0||t>=360))return false;
  if(_a.ref==ADCS_YAW&&(t < -180 || t > 180))return false;   // [Team] relative yaw
  _a.target=t;
  return true;
}

// [PIO port] Mission advance: load the next target while staying in AUTO
// (no rwStop / re-prepare between targets). Same range checks as adcsTarget().
inline bool adcsRetarget(float t){
  if(_a.ref==ADCS_SUN&&(t < -90 || t > 90))return false;
  if(_a.ref==ADCS_MAG&&(t<0||t>=360))return false;
  if(_a.ref==ADCS_YAW&&(t < -180 || t > 180))return false;   // [Team] relative yaw
  _a.target=t;
  return true;
}

inline bool adcsTune(float kp,float kd,int bias){
  if(kp<0||kp>ADCS_KP_MAX||kd<0||kd>ADCS_KD_MAX||bias<0||bias>100)return false;
  _a.kp=kp;_a.kd=kd;_a.bias=bias;return true;
}

// [Team] One step of the yaw EKF on the IMU sample the ADCS has just read. No extra I2C traffic:
// imuReadRaw() already fetched the accelerometer bytes every cycle and did not use them.
static unsigned long _aYawLastUs = 0;
static bool _aYawHave = false;
static int16_t _aYawMx = 0, _aYawMy = 0, _aYawMz = 0;
static float _aYawDt = 0;
inline void adcsYawEkfRun(const IMURawSample &r, const IMUProcessedSample &p){
  unsigned long t0 = micros();
  YawEkfInput in;
  in.dt = _aYawHave ? (float)(t0 - _aYawLastUs) * 1.0e-6f : 0.0f;
  _aYawLastUs = t0; _aYawDt = in.dt;
  in.rateDps = p.bodyRate;
  // the two gyro axes that are not the yaw axis: they only move when the body swings or is handled
  if (IMU_BODY_RATE_AXIS == IMU_AXIS_X)      { in.swingXDps = p.gyroY; in.swingYDps = p.gyroZ; }
  else if (IMU_BODY_RATE_AXIS == IMU_AXIS_Y) { in.swingXDps = p.gyroX; in.swingYDps = p.gyroZ; }
  else                                       { in.swingXDps = p.gyroX; in.swingYDps = p.gyroY; }
  in.magX = p.magX; in.magY = p.magY; in.magZ = p.magZ;
  // the magnetometer has its own 50 Hz clock: identical counts = no new sample since the last cycle
  in.magNew = !_aYawHave || r.mx != _aYawMx || r.my != _aYawMy || r.mz != _aYawMz;
  _aYawMx = r.mx; _aYawMy = r.my; _aYawMz = r.mz; _aYawHave = true;
  in.accX = (float)r.ax; in.accY = (float)r.ay; in.accZ = (float)r.az; in.accValid = true;
  yawEkfStep(in);
  yawEkfNoteExecMicros((uint32_t)(micros() - t0));
}

inline bool adcsRead(){
  IMURawSample r;
  IMUProcessedSample p;
  SunSample s;
  // [Team] same order and same failure behaviour as before; the EKF step sits between the two reads
  bool imuOk = imuReadRaw(r) && imuProcess(r,p);
  if(imuOk) adcsYawEkfRun(r,p);      // runs whatever the active reference is, so it is converged when selected
  if(!imuOk || !sunSensorRead(s)){
    _a.valid=false;
    return false;
  }

  _a.sunError = _a.target - s.angleDeg;
  _a.magError = adcsWrap180(_a.target - p.heading);
  _a.rate = p.bodyRate;

  // [Team] ADCS_REFERENCE,YAW: the EKF is the estimator. EST_RAW / EST_FILTERED show the
  // magnetometer's own relative heading, EST_ANGLE the EKF's, so the two can be compared live.
  if(_a.ref==ADCS_YAW){
    YawEkfOut y = yawEkfGet();
    if(!y.valid){ _a.valid=false; return false; }
    _a.rate = y.rateDps;                       // gyro rate with the estimated bias removed
    _a.rawReference = y.magRelDeg;
    _a.filteredReference = y.magRelDeg;
    _a.estimatedAngle = y.relDeg;
    estimatorSetExternal(y.magRelDeg, y.magRelDeg, y.relDeg, y.rateDps, _aYawDt);
    _a.error = adcsWrap180(_a.target - _a.estimatedAngle);
    _a.valid=true;
    return true;
  }

  _a.rawReference = (_a.ref==ADCS_MAG) ? p.heading : s.angleDeg;
  EstimatorReference er = (_a.ref==ADCS_MAG) ? EST_REF_MAG : EST_REF_SUN;
  if(!estimatorUpdate(_a.rawReference, _a.rate, er)){
    _a.valid=false;
    return false;
  }

  EstimatorState es = estimatorGet();
  _a.filteredReference = es.filteredReferenceDeg;
  _a.estimatedAngle = es.estimatedAngleDeg;

  // Controller now consumes the estimator output.
  // With LPF=OFF and Fusion=OFF this is equivalent to the verified v0.2 path.
  _a.error = (_a.ref==ADCS_MAG)
    ? adcsWrap180(_a.target - _a.estimatedAngle)
    : (_a.target - _a.estimatedAngle);

  _a.valid=true;
  return true;
}

inline bool adcsAuto(){
  if(!adcsRead())return false;
  if(rwGetMode()==RW_MODE_MOMENTUM && !rwMomentumProfileReady())return false;
  rwStop();
  if(rwGetMode()==RW_MODE_MOMENTUM && !rwApplyProfileBias())return false;
  _a.mode=ADCS_AUTO; _aLast=0; return true;
}

inline void adcsFault(){rwStop();_a.mode=ADCS_MANUAL;_a.u=0;_a.valid=false;}

inline void adcsUpdate(){
  unsigned long n=millis(); if(n-_aLast<ADCS_CONTROL_PERIOD_MS)return; _aLast=n;
  if(!adcsRead()){ if(_a.mode==ADCS_AUTO)adcsFault(); else _a.valid=false; return; }
  if(_a.mode!=ADCS_AUTO)return;

  float u=0; if(fabsf(_a.error)>=ADCS_DEADBAND_DEG)u=_a.kp*_a.error-_a.kd*_a.rate; _a.u=u;

  if(rwGetMode()==RW_MODE_REACTION){
    int c=(int)roundf(constrain(ADCS_CONTROL_SIGN*u,-(float)ADCS_MAX_RW_COMMAND,(float)ADCS_MAX_RW_COMMAND));
    rwSetReactionCommand(c); return;
  }

  // v3.0 profile-based Momentum AUTO.
  // The PD output is a CONTROL DEMAND; the strategy layer chooses a characterized maneuver.
  if(!rwMomentumProfileReady()){adcsFault();return;}
  if(rwIsMomentumBusy())return; // never restart an assist/recovery every 20 ms
  // Each characterized maneuver begins from nominal BIAS. After a maneuver,
  // recover first; do not stack repeated Assist pulses from a non-bias target.
  if(rwGetState()==RW_MOMENTUM_CONTROL_TARGET){rwStartRecovery();return;}

  MomentumProfile mp=rwMomentumProfileGet();
  float demand=ADCS_CONTROL_SIGN*u;
  bool settled=(fabsf(_a.error)<ADCS_DEADBAND_DEG && fabsf(_a.rate)<ADCS_RATE_DEADBAND_DPS);

  if(settled || fabsf(demand)<mp.trigger){ rwStartRecovery(); return; }
  if(demand>=mp.trigger){ rwStartProfileCW(); return; }
  if(demand<=-mp.trigger){ rwStartProfileCCW(); return; }
}
