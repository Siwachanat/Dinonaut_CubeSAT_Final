#pragma once

/* SunSeek Platform v2.0 — SYSTEM LAYER */

#include <Arduino.h>
#include "System_TTC.h"
#include "Module_IMU.h"
#include "Module_SunSensor.h"
#include "Module_ReactionWheel.h"
#include "Module_ADCS.h"
#include "Module_Estimator.h"
#include "System_TelemetryManager.h"

/*
  SensorTelemetry.h — T03 v0.3

  Two telemetry surfaces are intentionally separated:

  1) GS-compatible spacecraft telemetry
     Uses the exact KEY,VALUE names parsed by Ground Station v1.8:
       SUN_L, SUN_R, SUN_NDV, SUN_ANGLE, SUN_ERROR
       MAG_X, MAG_Y, MAG_Z, MAG_HEADING
       GYRO_Z, RW_CMD

     These packets drive SPACECRAFT SENSOR STATUS and SPACECRAFT RESPONSE.

  2) Engineering/raw telemetry
     Used in the Engineering terminal for characterization.  The GS does not
     need dedicated widgets for these fields.
*/

static bool _streamGyro = false;
static bool _streamAccel = false;
static bool _streamMag = false;
static bool _streamSun = false;
static bool _streamRaw = true;
static unsigned long _lastSensorStreamMs = 0;

// -------------------------------------------------------------------------
// T03 Calibration Assistant state
// -------------------------------------------------------------------------
static bool _magCalActive = false;
static int16_t _magMinX = 32767, _magMinY = 32767, _magMinZ = 32767;
static int16_t _magMaxX = -32768, _magMaxY = -32768, _magMaxZ = -32768;
static unsigned long _magCalSamples = 0;
static unsigned long _lastMagCalSampleMs = 0;

inline bool sensorGyroOffsetAssistant(uint16_t samples = 300, uint16_t sampleDelayMs = 10) {
  if (!imuGyroReady()) { sendTelemetry("ERR,GYRO_OFFSET,GYRO_NOT_READY"); return false; }

  sendTelemetry("ACK,GYRO_OFFSET");
  sendTelemetry("EVT,CAL,GYRO_OFFSET,KEEP_SPACECRAFT_STILL");
  sendTelemetry("EVT,CAL,GYRO_OFFSET,COLLECTING");

  double sx = 0.0, sy = 0.0, sz = 0.0;
  uint16_t valid = 0;
  for (uint16_t i = 0; i < samples; ++i) {
    IMURawSample r;
    if (imuReadRaw(r)) {
      sx += (double)r.gx * GYRO_SENSITIVITY_DPS_PER_LSB;
      sy += (double)r.gy * GYRO_SENSITIVITY_DPS_PER_LSB;
      sz += (double)r.gz * GYRO_SENSITIVITY_DPS_PER_LSB;
      ++valid;
    }
    delay(sampleDelayMs);
  }

  if (valid < samples * 9 / 10) {
    sendTelemetry("ERR,GYRO_OFFSET,INSUFFICIENT_SAMPLES," + String(valid));
    return false;
  }

  float bx = (float)(sx / valid);
  float by = (float)(sy / valid);
  float bz = (float)(sz / valid);
  sendTelemetry("EVT,CAL,GYRO_OFFSET,COMPLETE");
  sendTelemetry(
    "TM,CAL_GYRO_OFFSET,X," + String(bx, 4) +
    ",Y," + String(by, 4) +
    ",Z," + String(bz, 4) +
    ",SAMPLES," + String(valid)
  );
  sendTelemetry("EVT,CAL,SAVE_VALUES_TO_CONFIG_IMU");
  return true;
}

inline bool sensorMagCalStart() {
  if (!imuLSMReady()) { sendTelemetry("ERR,MAG_CAL,MAG_NOT_READY"); return false; }
  if (_magCalActive) { sendTelemetry("ERR,MAG_CAL,ALREADY_ACTIVE"); return false; }

  _magMinX = _magMinY = _magMinZ = 32767;
  _magMaxX = _magMaxY = _magMaxZ = -32768;
  _magCalSamples = 0;
  _lastMagCalSampleMs = 0;
  _magCalActive = true;
  sendTelemetry("ACK,MAG_CAL_START");
  sendTelemetry("EVT,CAL,MAG,ROTATE_SPACECRAFT_SLOWLY");
  sendTelemetry("EVT,CAL,MAG,COLLECTING");
  return true;
}

inline void sensorMagCalUpdate() {
  if (!_magCalActive) return;
  unsigned long now = millis();
  if (now - _lastMagCalSampleMs < 50UL) return;
  _lastMagCalSampleMs = now;

  IMURawSample r;
  if (!imuReadRaw(r)) return;
  if (r.mx < _magMinX) _magMinX = r.mx; if (r.mx > _magMaxX) _magMaxX = r.mx;
  if (r.my < _magMinY) _magMinY = r.my; if (r.my > _magMaxY) _magMaxY = r.my;
  if (r.mz < _magMinZ) _magMinZ = r.mz; if (r.mz > _magMaxZ) _magMaxZ = r.mz;
  ++_magCalSamples;
}

inline bool sensorMagCalStop() {
  if (!_magCalActive) { sendTelemetry("ERR,MAG_CAL,NOT_ACTIVE"); return false; }
  _magCalActive = false;
  sendTelemetry("ACK,MAG_CAL_STOP");

  if (_magCalSamples < 20) {
    sendTelemetry("ERR,MAG_CAL,INSUFFICIENT_SAMPLES," + String(_magCalSamples));
    return false;
  }

  float ox = ((float)_magMinX + (float)_magMaxX) * 0.5f;
  float oy = ((float)_magMinY + (float)_magMaxY) * 0.5f;
  float oz = ((float)_magMinZ + (float)_magMaxZ) * 0.5f;
  float hx = ((float)_magMaxX - (float)_magMinX) * 0.5f;
  float hy = ((float)_magMaxY - (float)_magMinY) * 0.5f;
  float hz = ((float)_magMaxZ - (float)_magMinZ) * 0.5f;
  float havg = (hx + hy + hz) / 3.0f;
  float sx = hx > 1.0f ? havg / hx : 1.0f;
  float sy = hy > 1.0f ? havg / hy : 1.0f;
  float sz = hz > 1.0f ? havg / hz : 1.0f;

  sendTelemetry("EVT,CAL,MAG,COMPLETE");
  sendTelemetry(
    "TM,CAL_MAG_MINMAX,X_MIN," + String(_magMinX) +
    ",X_MAX," + String(_magMaxX) +
    ",Y_MIN," + String(_magMinY) +
    ",Y_MAX," + String(_magMaxY) +
    ",Z_MIN," + String(_magMinZ) +
    ",Z_MAX," + String(_magMaxZ)
  );
  sendTelemetry(
    "TM,CAL_MAG_OFFSET,X," + String(ox, 2) +
    ",Y," + String(oy, 2) +
    ",Z," + String(oz, 2) +
    ",SAMPLES," + String(_magCalSamples)
  );
  sendTelemetry(
    "TM,CAL_MAG_SCALE,X," + String(sx, 4) +
    ",Y," + String(sy, 4) +
    ",Z," + String(sz, 4)
  );
  sendTelemetry("EVT,CAL,SAVE_VALUES_TO_CONFIG_IMU");
  return true;
}

inline bool sensorMagCalActive() { return _magCalActive; }

inline void sensorSendHealth() {
  // Human-readable Engineering telemetry; not consumed by the v1.8 sensor panel.
  sendTelemetry(String("TM,SENSOR_ACCEL,") + (imuLSMReady() ? "READY" : "NOT_DETECTED"));
  sendTelemetry(String("TM,SENSOR_MAG,") + (imuLSMReady() ? "READY" : "NOT_DETECTED"));
  sendTelemetry(String("TM,SENSOR_GYRO,") + (imuGyroReady() ? "READY" : "NOT_DETECTED"));
  sendTelemetry(String("TM,SENSOR_BARO,") + (imuBaroDetected() ? "DETECTED" : "NOT_DETECTED"));
  sendTelemetry("TM,SENSOR_SUN,READY");
}

inline void sensorSendIMUStatus() {
  sendTelemetry(
    String("TM,IMU_LSM303D,") + (imuLSMReady() ? "READY" : "NOT_DETECTED") +
    ",IMU_ADDR,0x1D,GYRO_NAME," + imuGyroName() +
    ",GYRO_ADDR,0x" + String(imuGyroAddress(), HEX) +
    ",GYRO_WHO_EXPECTED,0x" + String(imuGyroWhoExpected(), HEX) +
    ",GYRO_STATUS," + (imuGyroReady() ? "READY" : "NOT_DETECTED")
  );
}

inline void sensorSendCalibrationStatus() {
  sendTelemetry(
    "TM,GYRO_BIAS_X," + String(GYRO_BIAS_X_DPS, 3) +
    ",GYRO_BIAS_Y," + String(GYRO_BIAS_Y_DPS, 3) +
    ",GYRO_BIAS_Z," + String(GYRO_BIAS_Z_DPS, 3)
  );
  sendTelemetry(
    "TM,BODY_RATE_AXIS," + String(IMU_BODY_RATE_AXIS) +
    ",BODY_RATE_SIGN," + String(IMU_BODY_RATE_SIGN, 1)
  );
  sendTelemetry(
    "TM,MAG_OFFSET_X," + String(MAG_OFFSET_X, 2) +
    ",MAG_OFFSET_Y," + String(MAG_OFFSET_Y, 2) +
    ",MAG_OFFSET_Z," + String(MAG_OFFSET_Z, 2)
  );
  sendTelemetry(
    "TM,MAG_SCALE_X," + String(MAG_SCALE_X, 4) +
    ",MAG_SCALE_Y," + String(MAG_SCALE_Y, 4) +
    ",MAG_SCALE_Z," + String(MAG_SCALE_Z, 4)
  );
#if SUN_ALGORITHM == 1
  // [Team] regional-round method; tables from Config_SunCal.h
  sendTelemetry(String("TM,SUN_ESTIMATOR,TEAM_RATIO_LEVEL,SUN_CAL,") + SUNCAL_LABEL);
#else
  sendTelemetry("TM,SUN_ESTIMATOR,TEAM_FUNCTION");
#endif
}

inline bool sensorReadIMU(IMURawSample &raw, IMUProcessedSample &cal) {
  if (!imuReadRaw(raw)) return false;
  return imuProcess(raw, cal);
}

inline bool sensorReadAll(IMURawSample &raw, IMUProcessedSample &imu, SunSample &sun) {
  bool imuOK = sensorReadIMU(raw, imu);
  bool sunOK = sunSensorRead(sun);
  return imuOK && sunOK;
}

inline void sensorSendGSSnapshot() {
  IMURawSample r; IMUProcessedSample p; SunSample s;
  if (!sensorReadAll(r, p, s)) {
    sendTelemetry("ERR,SENSOR_SNAPSHOT_READ_FAILED");
    return;
  }

  // IMPORTANT: Keep this packet below the preferred BLE MTU and keep the
  // key names aligned with Ground Station v1.8 _parse().
  sendTelemetry(
    "TM,SUN_L," + String(s.leftRaw) +
    ",SUN_R," + String(s.rightRaw) +
    ",SUN_NDV," + String(s.ndv, 4) +
    ",SUN_ANGLE," + String(s.angleDeg, 2) +
    ",SUN_ERROR," + String(s.errorDeg, 2) +
    ",MAG_X," + String(p.magX, 2) +
    ",MAG_Y," + String(p.magY, 2) +
    ",MAG_Z," + String(p.magZ, 2) +
    ",MAG_HEADING," + String(p.heading, 2) +
    ",GYRO_Z," + String(p.bodyRate, 3) +
    ",RW_CMD," + String(rwGetMotorCommand())
  );
}

inline void sensorSendRawSnapshot() {
  IMURawSample r; IMUProcessedSample p; SunSample s;
  if (!sensorReadAll(r, p, s)) {
    sendTelemetry("ERR,SENSOR_RAW_READ_FAILED");
    return;
  }

  // Raw data remains Engineering telemetry.  It is intentionally split so
  // each BLE notification stays compact.
  sendTelemetry(
    "TM,RAW_GX," + String(r.gx) +
    ",RAW_GY," + String(r.gy) +
    ",RAW_GZ," + String(r.gz) +
    ",RAW_AX," + String(r.ax) +
    ",RAW_AY," + String(r.ay) +
    ",RAW_AZ," + String(r.az)
  );
  sendTelemetry(
    "TM,RAW_MX," + String(r.mx) +
    ",RAW_MY," + String(r.my) +
    ",RAW_MZ," + String(r.mz) +
    ",RAW_SUN_L," + String(s.leftRaw) +
    ",RAW_SUN_R," + String(s.rightRaw)
  );
}

inline void sensorSendGyroOnce(bool includeRaw = true) {
  IMURawSample r; IMUProcessedSample p;
  if (!sensorReadIMU(r, p)) { sendTelemetry("ERR,GYRO_READ_FAILED"); return; }
  if (includeRaw) {
    sendTelemetry("TM,RAW_GX," + String(r.gx) + ",RAW_GY," + String(r.gy) + ",RAW_GZ," + String(r.gz));
  }
  // GYRO_Z is the spacecraft body-rate quantity consumed by GS v1.8.
  sendTelemetry("TM,GYRO_Z," + String(p.bodyRate, 3));
}

inline void sensorSendAccelOnce(bool includeRaw = true) {
  IMURawSample r; IMUProcessedSample p;
  if (!sensorReadIMU(r, p)) { sendTelemetry("ERR,ACC_READ_FAILED"); return; }
  sendTelemetry("TM,RAW_AX," + String(r.ax) + ",RAW_AY," + String(r.ay) + ",RAW_AZ," + String(r.az));
}

inline void sensorSendMagOnce(bool includeRaw = true) {
  IMURawSample r; IMUProcessedSample p;
  if (!sensorReadIMU(r, p)) { sendTelemetry("ERR,MAG_READ_FAILED"); return; }
  if (includeRaw) {
    sendTelemetry("TM,RAW_MX," + String(r.mx) + ",RAW_MY," + String(r.my) + ",RAW_MZ," + String(r.mz));
  }
  sendTelemetry(
    "TM,MAG_X," + String(p.magX, 2) +
    ",MAG_Y," + String(p.magY, 2) +
    ",MAG_Z," + String(p.magZ, 2) +
    ",MAG_HEADING," + String(p.heading, 2)
  );
}

inline void sensorSendSunOnce(bool includeRaw = true) {
  SunSample s;
  if (!sunSensorRead(s)) { sendTelemetry("ERR,SUN_READ_FAILED"); return; }
  if (includeRaw) {
    sendTelemetry("TM,RAW_SUN_L," + String(s.leftRaw) + ",RAW_SUN_R," + String(s.rightRaw));
  }
  sendTelemetry(
    "TM,SUN_L," + String(s.leftRaw) +
    ",SUN_R," + String(s.rightRaw) +
    ",SUN_NDV," + String(s.ndv, 4) +
    ",SUN_ANGLE," + String(s.angleDeg, 2) +
    ",SUN_ERROR," + String(s.errorDeg, 2)
  );
}

// [Team] SUN_STATUS: what the sun-angle algorithm is doing right now.
// Line 1 = algorithm state, line 2 = ADC health and the counts-at-3V3 estimate
// to pass to tools/suncal_fit.py --vcc-counts.
inline void sensorSendSunStatus() {
  SunSample s;
  if (!sunSensorRead(s)) { sendTelemetry("ERR,SUN_READ_FAILED"); return; }
  sendTelemetry(
    "TM,SUN_ALG," + String(SUN_ALGORITHM) +
    ",SUN_BRANCH," + String(sunBranchText(s.branch)) +
    ",SUN_ANGLE," + String(s.angleDeg, 2) +
    ",SUN_X," + String(s.x, 4) +
    ",SUN_LNSUM," + String(s.lnSum, 4) +
    ",SUN_REF," + String(sunAmpRefValid() ? 1 : 0) +
    ",SUN_AMPREF," + String(sunAmpRef(), 3) +
    ",SUN_SAT," + String(s.saturated ? 1 : 0)
  );
  float mvL = 0, mvR = 0;
  sunReadMilliVolts(mvL, mvR);
  // counts the 3V3 rail would read, from the live counts-per-millivolt of each channel
  float est = 0; int k = 0;
  if (mvL > 100.0f) { est += s.leftAvg * 3300.0f / mvL; ++k; }
  if (mvR > 100.0f) { est += s.rightAvg * 3300.0f / mvR; ++k; }
  sendTelemetry(
    "TM,SUN_AVG_L," + String(s.leftAvg, 1) +
    ",SUN_AVG_R," + String(s.rightAvg, 1) +
    ",SUN_MV_L," + String(mvL, 0) +
    ",SUN_MV_R," + String(mvR, 0) +
    ",SUN_VCC_COUNTS_EST," + String(k ? est / k : 0.0f, 0) +
    ",SUN_OVERSAMPLE," + String(SUN_OVERSAMPLE) +
    ",SUN_READ_US," + String(sunLastReadMicros())
  );
}

inline void sensorSetAllStreams(bool on) {
  _streamGyro = on;
  _streamAccel = on;
  _streamMag = on;
  _streamSun = on;
}

inline bool sensorAnyStreamEnabled() {
  return _streamGyro || _streamAccel || _streamMag || _streamSun;
}

inline void sensorUpdate() {
  sensorMagCalUpdate();
  if (!sensorAnyStreamEnabled()) return;

  unsigned long now = millis();
  if (now - _lastSensorStreamMs < SENSOR_STREAM_PERIOD_MS) return;
  _lastSensorStreamMs = now;

  // The GS-compatible snapshot is always emitted while a sensor stream is
  // active, so the v1.8 panels stay live during characterization.
  sensorSendGSSnapshot();

  // RAW mode adds Engineering counts for calibration work. CAL mode keeps
  // only the compact GS-compatible physical quantities.
  if (_streamRaw) sensorSendRawSnapshot();
}

// T04 operational telemetry. Flat key/value format for GS compatibility.
static unsigned long _lastAdcsTm=0;
inline void sensorSendEstimatorSnapshot(){
  EstimatorState e=estimatorGet();
  sendTelemetry(
    "TM,EST_RAW,"+String(e.rawReferenceDeg,2)+
    ",EST_FILTERED,"+String(e.filteredReferenceDeg,2)+
    ",EST_ANGLE,"+String(e.estimatedAngleDeg,2)+
    ",EST_FILTER,"+String(e.filterEnabled?"ON":"OFF")+
    ",EST_FILTER_TYPE,"+String(estimatorFilterTypeText())+
    ",EST_MA_WINDOW,"+String(e.movingAverageWindow)+
    ",EST_FILTER_STRENGTH,"+String(e.filterStrength,2)
  );
  sendTelemetry(
    "TM,EST_FUSION,"+String(e.fusionEnabled?"ON":"OFF")+
    ",EST_GYRO_WEIGHT,"+String(e.gyroWeight,2)+
    ",EST_VALID,"+String(e.valid?"1":"0")
  );
}
// [Team] Relative-yaw EKF telemetry (Module_YawEKF.h).
//   YAW_REL    relative yaw the controller uses, -180..+180, + = rotated right of the zero datum
//   YAW_ABS    estimated heading 0..360 (same sense as MAG_HEADING)
//   YAW_MAG    the magnetometer's own heading this cycle (compare with YAW_ABS)
//   YAW_BIAS   gyro bias the filter has found, on top of GYRO_BIAS_Z_DPS
//   YAW_SIGMA  1-sigma uncertainty of YAW_REL from the filter covariance
//   YAW_INNOV  last residual: magnetometer minus prediction
inline void sensorSendYawSnapshot(){
  YawEkfOut y=yawEkfGet();
  sendTelemetry(
    "TM,YAW_REL,"+String(y.relDeg,2)+
    ",YAW_ABS,"+String(y.absDeg,2)+
    ",YAW_MAG,"+String(y.magDeg,2)+
    ",YAW_BIAS,"+String(y.biasDps,3)+
    ",YAW_SIGMA,"+String(y.sigmaDeg,2)+
    ",YAW_INNOV,"+String(y.innovDeg,2)+
    ",YAW_ZEROED,"+String(y.zeroed?"1":"0")+
    ",YAW_VALID,"+String(y.valid?"1":"0")
  );
}
// YAW_STATUS: line 1 estimate, line 2 filter health, line 3 measurement quality / tilt / cost.
inline void sensorSendYawStatus(){
  YawEkfOut y=yawEkfGet(); YawEkfStatus s=yawEkfStatus();
  sensorSendYawSnapshot();
  sendTelemetry(
    "TM,YAW_ZERO_HEADING,"+String(s.zeroAbsDeg,2)+
    ",YAW_RATE,"+String(y.rateDps,3)+
    ",YAW_SIGMA_BIAS,"+String(s.sigmaBiasDps,3)+
    ",YAW_INNOV_RMS,"+String(s.innovRmsDeg,2)+
    ",YAW_GAIN,"+String(s.gainYaw,4)+
    ",YAW_USED,"+String(s.used)+
    ",YAW_REJECTED,"+String(s.rejected)+
    ",YAW_REACQUIRED,"+String(s.reacquired)
  );
  const char* tc = !s.tiltCompOn ? "OFF" : (s.tiltCompAllowed ? "ON" : "REFUSED");
  sendTelemetry(
    "TM,YAW_FIELD,"+String(s.field,2)+
    ",YAW_FIELD_REF,"+String(s.fieldRef,2)+
    ",YAW_VAR_SCALE,"+String(s.varScale,2)+
    ",TILT_DEG,"+String(s.tiltDeg,2)+
    ",ACC_OK,"+String(s.accOk?"1":"0")+
    ",TILT_COMP,"+String(tc)+
    ",YAW_MAG_SIGMA,"+String(s.magSigmaDeg,2)+
    ",EKF_US,"+String(s.execUs)+
    ",EKF_US_MAX,"+String(s.execUsMax)
  );
}
inline void sensorSendADCSSnapshot(){
  // Telemetry is an observer only. adcsUpdate() owns periodic sensor/estimator updates.
  ADCSState a=adcsGet();
  sendTelemetry(
    "TM,ADCS_MODE,"+String(adcsModeText())+
    ",ADCS_REFERENCE,"+String(adcsRefText())+
    ",TARGET,"+String(a.target,2)+
    ",POINTING_ERROR,"+String(a.error,2)+
    ",SUN_ERROR,"+String(a.sunError,2)+
    ",MAG_ERROR,"+String(a.magError,2)+
    ",GYRO_Z,"+String(a.rate,3)+
    ",RW_CMD,"+String(rwGetMotorCommand())
  );
  sensorSendEstimatorSnapshot();
  if(a.ref==ADCS_YAW) sensorSendYawSnapshot();   // [Team] one extra line, only while flying on the EKF
}
inline void sensorADCSUpdate(){if(!_tmStream.adcs)return;unsigned long n=millis();if(n-_lastAdcsTm<ADCS_TM_PERIOD_MS)return;_lastAdcsTm=n;sensorSendADCSSnapshot();}


// ---- T07 selective Engineering telemetry ----
inline void sensorSendSelectedSnapshot(bool sunOn,bool magOn,bool gyroOn){
  IMURawSample r; IMUProcessedSample p; SunSample ss;
  bool imuOk=imuReadRaw(r)&&imuProcess(r,p);
  bool sunOk=sunSensorRead(ss);

  if(sunOn){
    if(sunOk) sendTelemetry("TM,SUN_L,"+String(ss.leftRaw)+",SUN_R,"+String(ss.rightRaw)+",SUN_NDV,"+String(ss.ndv,4)+",SUN_ANGLE,"+String(ss.angleDeg,2));
    else sendTelemetry("TM,SUN,INVALID");
  }
  if(magOn){
    if(imuOk) sendTelemetry("TM,MAG_X,"+String(p.magX,2)+",MAG_Y,"+String(p.magY,2)+",MAG_Z,"+String(p.magZ,2)+",MAG_HEADING,"+String(p.heading,2));
    else sendTelemetry("TM,MAG,INVALID");
  }
  if(gyroOn){
    if(imuOk) sendTelemetry("TM,GYRO_X,"+String(p.gyroX,3)+",GYRO_Y,"+String(p.gyroY,3)+",GYRO_Z,"+String(p.gyroZ,3)+",BODY_RATE,"+String(p.bodyRate,3));
    else sendTelemetry("TM,GYRO,INVALID");
  }
}

inline void sensorTelemetryUpdate(){
  if(!telemetryStreamDue()) return;
  TelemetryStreamState t=telemetryStreamGet();
  sensorSendSelectedSnapshot(t.sun,t.mag,t.gyro);
}
