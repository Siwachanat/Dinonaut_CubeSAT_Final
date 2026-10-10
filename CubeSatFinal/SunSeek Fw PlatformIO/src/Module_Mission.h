#pragma once

/* SunSeek Platform v2.0 — MODULE LAYER */
#include <Arduino.h>
#include <math.h>
#include "Module_ADCS.h"
#include "Module_ReactionWheel.h"
#include "Module_Payload.h"     // [PIO port] CAPTURE / IMAGE_READY transaction
#include "System_TTC.h"         // [PIO port] EVT lines
#include "Config_Mission.h"     // [PIO port] advance settings

enum MissionState {
  MISSION_IDLE,
  MISSION_PREPARING,
  MISSION_READY,
  MISSION_ACQUIRING,
  MISSION_HOLDING,
  MISSION_CAPTURING,
  MISSION_IMAGE_READY,
  MISSION_COMPLETE,
  MISSION_ABORTED,
  MISSION_FAILED,
  MISSION_TIME_LIMIT_REACHED
};

struct MissionTarget {
  float angleDeg;
  float toleranceDeg;
  float holdSec;
  bool capture;   // [Team 2026-10-08] take a picture when this target has been held (MISSION_TARGET 4th field)
};

#define MISSION_MAX_TARGETS 10

struct MissionRuntime {
  MissionState state;
  MissionTarget targets[MISSION_MAX_TARGETS];
  uint8_t targetCount;
  uint8_t currentIndex;
  unsigned long startMs;
  unsigned long holdStartMs;
  unsigned long maxDurationMs;
  bool timeLimitReached;
  // [PIO port] mission advance
  unsigned long captureStartMs;
  uint8_t captureAttempts;
  unsigned long nextTargetAtMs;
  uint8_t imagesCaptured;
};

static MissionRuntime _mission = {MISSION_IDLE, {}, 0, 0, 0, 0, 5UL*60UL*1000UL, false, 0, 0, 0, 0};

inline const char* missionStateText(){
  switch(_mission.state){
    case MISSION_IDLE:return "IDLE";
    case MISSION_PREPARING:return "PREPARING";
    case MISSION_READY:return "READY";
    case MISSION_ACQUIRING:return "ACQUIRING";
    case MISSION_HOLDING:return "HOLDING";
    case MISSION_CAPTURING:return "CAPTURING";
    case MISSION_IMAGE_READY:return "IMAGE_READY";
    case MISSION_COMPLETE:return "COMPLETE";
    case MISSION_ABORTED:return "ABORTED";
    case MISSION_FAILED:return "FAILED";
    case MISSION_TIME_LIMIT_REACHED:return "TIME_LIMIT_REACHED";
  }
  return "UNKNOWN";
}

// [PIO port] every state change is announced: EVT,MISSION_STATE,<state>,<targetIndex 1-based>
inline void _missionSetState(MissionState st){
  if(_mission.state==st) return;
  _mission.state=st;
  sendTelemetry("EVT,MISSION_STATE,"+String(missionStateText())+","+String(_mission.currentIndex+1));
}

inline void missionBegin(){ _mission.state=MISSION_IDLE; }

inline bool missionSetMaxMinutes(uint16_t minutes){
  if(!((minutes>=1 && minutes<=15)||minutes==20||minutes==30)) return false;
  if(_mission.state!=MISSION_IDLE && _mission.state!=MISSION_READY) return false;
  _mission.maxDurationMs=(unsigned long)minutes*60UL*1000UL;
  return true;
}

inline bool missionClearTargets(){
  if(_mission.state!=MISSION_IDLE) return false;
  _mission.targetCount=0; _mission.currentIndex=0; return true;
}

// [Team 2026-10-08] capture is chosen per target. Without the 4th telecommand field it is
// MISSION_CAPTURE_ENABLE (Config_Mission.h), i.e. exactly the behaviour before this change.
inline bool missionAddTarget(float angle,float tolerance,float holdSec,bool capture=(MISSION_CAPTURE_ENABLE!=0)){
  if(_mission.state!=MISSION_IDLE || _mission.targetCount>=MISSION_MAX_TARGETS) return false;
  if(!isfinite(angle)||!isfinite(tolerance)||!isfinite(holdSec)) return false;
  if(tolerance<=0 || tolerance>30 || holdSec<0 || holdSec>60) return false;
  _mission.targets[_mission.targetCount++]={angle,tolerance,holdSec,capture};
  return true;
}

inline bool missionPrepare(){
  if(_mission.state!=MISSION_IDLE || _mission.targetCount<1) return false;
  _mission.state=MISSION_PREPARING;

  // v0.1: Reaction is immediately ready after configuration checks.
  // Momentum preparation state is exposed now; dynamic steady-state qualification
  // will be hardware-characterized before it becomes an automatic READY criterion.
  _mission.state=MISSION_READY;
  return true;
}

inline bool missionStart(){
  if(_mission.state!=MISSION_READY || _mission.targetCount<1) return false;
  _mission.currentIndex=0;
  _mission.startMs=millis();
  _mission.holdStartMs=0;
  _mission.timeLimitReached=false;
  _mission.captureStartMs=0; _mission.captureAttempts=0; _mission.nextTargetAtMs=0; _mission.imagesCaptured=0;
  _missionSetState(MISSION_ACQUIRING);
  return true;
}

inline void missionAbort(){
  if(_mission.state==MISSION_IDLE) return;
  adcsManual(); rwStop(); _missionSetState(MISSION_ABORTED);
}

inline void missionReset(){
  adcsManual(); rwStop();
  _mission.state=MISSION_IDLE;
  _mission.currentIndex=0;
  _mission.startMs=0; _mission.holdStartMs=0; _mission.timeLimitReached=false;
  _mission.captureStartMs=0; _mission.captureAttempts=0; _mission.nextTargetAtMs=0; _mission.imagesCaptured=0;
}

inline MissionRuntime missionGet(){return _mission;}

// [PIO port] ---------------------------------------------------------------
// Advance to the next target without leaving AUTO; COMPLETE after the last one.
inline void _missionAdvance(){
  _mission.currentIndex++;
  _mission.holdStartMs=0; _mission.captureStartMs=0; _mission.captureAttempts=0;
  if(_mission.currentIndex>=_mission.targetCount){
    sendTelemetry("EVT,MISSION_COMPLETE,"+String(_mission.imagesCaptured)+","+String((millis()-_mission.startMs)/1000UL));
    _missionSetState(MISSION_COMPLETE);          // ADCS stays AUTO on the last target (no auto RW STOP)
    return;
  }
  MissionTarget &t=_mission.targets[_mission.currentIndex];
  if(!adcsRetarget(t.angleDeg)){ _missionSetState(MISSION_FAILED); return; }
  sendTelemetry("EVT,MISSION_NEXT_TARGET,"+String(_mission.currentIndex+1)+","+String(t.angleDeg,2));
  _missionSetState(MISSION_ACQUIRING);
}

inline void _missionStartCapture(){
  _mission.captureAttempts++;
  _mission.captureStartMs=millis();
  payloadRequestCapture();
  sendTelemetry("EVT,MISSION_CAPTURE,"+String(_mission.currentIndex+1)+","+String(_mission.captureAttempts));
}

inline void missionUpdate(){
  MissionState st=_mission.state;
  if(st!=MISSION_ACQUIRING && st!=MISSION_HOLDING && st!=MISSION_CAPTURING && st!=MISSION_IMAGE_READY) return;

  unsigned long now=millis();
  if(!_mission.timeLimitReached && now-_mission.startMs>=_mission.maxDurationMs){
    _mission.timeLimitReached=true;
    _missionSetState(MISSION_TIME_LIMIT_REACHED);
    // IMPORTANT: competition rule — do NOT auto-abort and do NOT auto RW STOP.
    return;
  }

  // ---- capture transaction (ADCS keeps holding the target in AUTO) ----
  if(st==MISSION_CAPTURING){
    if(payloadTakeImageReady()){
      _mission.imagesCaptured++;
      sendTelemetry("EVT,MISSION_IMAGE,"+String(_mission.currentIndex+1)+","+payloadLastImageName()+","+String(payloadLastImageBytes()));
      _mission.nextTargetAtMs=now+MISSION_NEXT_TARGET_DELAY_MS;
      _missionSetState(MISSION_IMAGE_READY);
      return;
    }
    bool failed=payloadTakeCaptureError() || (now-_mission.captureStartMs>=MISSION_CAPTURE_TIMEOUT_MS);
    if(!failed) return;
    if(_mission.captureAttempts<=MISSION_CAPTURE_RETRIES){ _missionStartCapture(); return; }
    sendTelemetry("EVT,MISSION_CAPTURE_FAILED,"+String(_mission.currentIndex+1));
    if(MISSION_CONTINUE_ON_CAPTURE_FAIL){ _mission.nextTargetAtMs=now; _missionSetState(MISSION_IMAGE_READY); }
    else _missionSetState(MISSION_FAILED);
    return;
  }
  if(st==MISSION_IMAGE_READY){
    if((long)(now-_mission.nextTargetAtMs)>=0) _missionAdvance();
    return;
  }

  if(_mission.currentIndex>=_mission.targetCount){
    _missionSetState(MISSION_COMPLETE); return;
  }

  MissionTarget &t=_mission.targets[_mission.currentIndex];

  // Mission Manager configures the same verified ADCS path used by Operation.
  if(adcsGet().mode!=ADCS_AUTO){
    if(!adcsTarget(t.angleDeg) || !adcsAuto()){
      _missionSetState(MISSION_FAILED); return;
    }
  }

  ADCSState a=adcsGet();
  if(!a.valid) return;

  if(fabsf(a.error)<=t.toleranceDeg){
    if(_mission.state!=MISSION_HOLDING){
      _missionSetState(MISSION_HOLDING);
      _mission.holdStartMs=now;
    } else if(now-_mission.holdStartMs >= (unsigned long)(t.holdSec*1000.0f)){
      sendTelemetry("EVT,MISSION_TARGET_HELD,"+String(_mission.currentIndex+1)+","+String(a.error,2));
      _missionSetState(MISSION_CAPTURING);
      if(t.capture){ _missionStartCapture(); }
      else {   // [Team] no picture wanted at this target: say so and move on
        sendTelemetry("EVT,MISSION_CAPTURE_SKIPPED,"+String(_mission.currentIndex+1));
        _mission.nextTargetAtMs=now+MISSION_NEXT_TARGET_DELAY_MS; _missionSetState(MISSION_IMAGE_READY);
      }
    }
  } else {
    if(_mission.state==MISSION_HOLDING) _missionSetState(MISSION_ACQUIRING);
    _mission.holdStartMs=0;
  }
}
