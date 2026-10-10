#pragma once

/* SunSeek Platform v2.0 — SYSTEM LAYER */
#include <Arduino.h>

/*
  T07 v0.1 Telemetry Manager

  Sensor acquisition and sensor downlink are deliberately separate.
  ADCS / estimator may continue reading sensors while Engineering sensor TM is OFF.

  Power-up default: all continuous Engineering sensor telemetry OFF.
*/

struct TelemetryStreamState {
  bool sun;
  bool mag;
  bool gyro;
  bool adcs;
  uint16_t rateHz;
};

static TelemetryStreamState _tmStream = {false, false, false, false, 5};
static unsigned long _tmLastMs = 0;

inline TelemetryStreamState telemetryStreamGet(){ return _tmStream; }

inline void telemetryStreamSetAll(bool on){
  _tmStream.sun=on; _tmStream.mag=on; _tmStream.gyro=on;
}

inline bool telemetryStreamSetRate(uint16_t hz){
  if(hz < 1 || hz > 20) return false;
  _tmStream.rateHz=hz; return true;
}

inline bool telemetryStreamDue(){
  if(!_tmStream.sun && !_tmStream.mag && !_tmStream.gyro) return false;
  unsigned long now=millis();
  unsigned long period=1000UL/_tmStream.rateHz;
  if(now-_tmLastMs < period) return false;
  _tmLastMs=now; return true;
}
