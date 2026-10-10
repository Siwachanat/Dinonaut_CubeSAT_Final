#pragma once

/* SunSeek Platform Firmware v3.0 — Reaction Wheel / Momentum Profile Layer
   Major change from v2.1:
   - separates nominal momentum bias from current target
   - adds runtime Momentum Control Profile
   - adds non-blocking CW/CCW assist and bias recovery
   - keeps legacy manual RW / RW_BIAS / RW_CMD behavior
*/
#include <Arduino.h>
#include "Config_Actuator.h"

enum RWMode { RW_MODE_REACTION, RW_MODE_MOMENTUM };
enum RWState {
  RW_STOPPED,
  RW_REACTION_DRIVE,
  RW_MOMENTUM_BIAS,
  RW_MOMENTUM_ASSIST_CW,
  RW_MOMENTUM_ASSIST_CCW,
  RW_MOMENTUM_CONTROL_TARGET,
  RW_MOMENTUM_RECOVERY
};

struct MomentumManeuverProfile {
  int delta;
  int assist;
  unsigned long durationMs;
  bool configured;
};

struct MomentumProfile {
  int bias;
  MomentumManeuverProfile cw;
  MomentumManeuverProfile ccw;
  int recoveryStep;
  unsigned long recoveryIntervalMs;
  float trigger;
  bool biasConfigured;
  bool recoveryConfigured;
  bool triggerConfigured;
};

static RWMode _rwMode = RW_MODE_REACTION;
static RWState _rwState = RW_STOPPED;
static int _rwMotorCommand = 0;
static int _rwNominalBias = 0;
static int _rwCurrentTarget = 0;
static int _rwDelta = 0;
static int _rwAssist = 0;
static unsigned long _rwDuration = 0;
static unsigned long _rwStateStart = 0;
static unsigned long _rwRecoveryLast = 0;
static bool _rwManeuverCompleteEvent = false;
static bool _rwBiasRecoveredEvent = false;
static MomentumProfile _rwProfile = {0,{0,0,0,false},{0,0,0,false},0,0,0.0f,false,false,false};

inline int _rwPercentToPWM(int percent){ percent=constrain(percent,0,100); return map(percent,0,100,0,255); }
inline void _rwDrive(int command){
  command=constrain(command,-100,100); _rwMotorCommand=command;
  int pwm=_rwPercentToPWM(abs(command));
  if(command==0){ analogWrite(RW_PIN_PWMA,0); digitalWrite(RW_PIN_AIN1,LOW); digitalWrite(RW_PIN_AIN2,LOW); }
  else if(command>0){ digitalWrite(RW_PIN_AIN1,HIGH); digitalWrite(RW_PIN_AIN2,LOW); analogWrite(RW_PIN_PWMA,pwm); }
  else { digitalWrite(RW_PIN_AIN1,LOW); digitalWrite(RW_PIN_AIN2,HIGH); analogWrite(RW_PIN_PWMA,pwm); }
}
inline void _rwResetRuntime(){
  _rwNominalBias=0; _rwCurrentTarget=0; _rwDelta=0; _rwAssist=0; _rwDuration=0;
  _rwStateStart=0; _rwRecoveryLast=0; _rwManeuverCompleteEvent=false; _rwBiasRecoveredEvent=false;
}
inline void rwBegin(){ pinMode(RW_PIN_PWMA,OUTPUT); pinMode(RW_PIN_AIN1,OUTPUT); pinMode(RW_PIN_AIN2,OUTPUT); _rwDrive(0); _rwResetRuntime(); }

inline void rwUpdate(){
  unsigned long now=millis();
  if((_rwState==RW_MOMENTUM_ASSIST_CW || _rwState==RW_MOMENTUM_ASSIST_CCW) && now-_rwStateStart>=_rwDuration){
    _rwDrive(_rwCurrentTarget); _rwState=RW_MOMENTUM_CONTROL_TARGET; _rwManeuverCompleteEvent=true;
  }
  if(_rwState==RW_MOMENTUM_RECOVERY && now-_rwRecoveryLast>=_rwProfile.recoveryIntervalMs){
    _rwRecoveryLast=now;
    int c=_rwMotorCommand;
    int step=max(1,_rwProfile.recoveryStep);
    if(c<_rwNominalBias) c=min(c+step,_rwNominalBias);
    else if(c>_rwNominalBias) c=max(c-step,_rwNominalBias);
    _rwDrive(c);
    if(c==_rwNominalBias){ _rwCurrentTarget=_rwNominalBias; _rwState=RW_MOMENTUM_BIAS; _rwBiasRecoveredEvent=true; }
  }
}
inline bool rwTakeManeuverCompleteEvent(){ if(!_rwManeuverCompleteEvent)return false; _rwManeuverCompleteEvent=false; return true; }
inline bool rwTakeBiasRecoveredEvent(){ if(!_rwBiasRecoveredEvent)return false; _rwBiasRecoveredEvent=false; return true; }
inline void rwSetMode(RWMode mode){ _rwDrive(0); _rwState=RW_STOPPED; _rwMode=mode; _rwResetRuntime(); }
inline void rwSetReactionCommand(int command){ _rwDrive(command); _rwState=(command==0)?RW_STOPPED:RW_REACTION_DRIVE; }
inline void rwStop(){ _rwDrive(0); _rwState=RW_STOPPED; _rwResetRuntime(); }

// Manual/prepare bias command. In v3.0 this is explicitly the nominal bias.
inline bool rwSetBias(int bias){
  if(bias<0||bias>100)return false;
  _rwNominalBias=bias; _rwCurrentTarget=bias; _rwDelta=0; _rwAssist=0; _rwDuration=0;
  _rwManeuverCompleteEvent=false; _rwBiasRecoveredEvent=false; _rwDrive(bias);
  _rwState=(bias==0)?RW_STOPPED:RW_MOMENTUM_BIAS; return true;
}

// Legacy/manual characterization primitive: target is relative to the CURRENT target.
// Nominal bias is intentionally preserved so recovery can still return to it.
inline bool rwMomentumCommand(int delta,int assist,unsigned long durationMs){
  long requested=(long)_rwCurrentTarget+delta;
  if(requested<0||requested>100||assist<-100||assist>100)return false;
  _rwDelta=delta; _rwCurrentTarget=(int)requested; _rwAssist=assist; _rwDuration=durationMs;
  _rwManeuverCompleteEvent=false;
  if(assist==0||durationMs==0){ _rwDrive(_rwCurrentTarget); _rwState=RW_MOMENTUM_CONTROL_TARGET; return true; }
  _rwDrive(assist); _rwStateStart=millis();
  _rwState=(delta>=0)?RW_MOMENTUM_ASSIST_CW:RW_MOMENTUM_ASSIST_CCW; return true;
}

inline void rwMomentumProfileClear(){ _rwProfile={0,{0,0,0,false},{0,0,0,false},0,0,0.0f,false,false,false}; }
inline bool rwMomentumProfileSetBias(int bias){ if(bias<0||bias>100)return false; _rwProfile.bias=bias; _rwProfile.biasConfigured=true; return true; }
inline bool rwMomentumProfileSetCW(int delta,int assist,unsigned long durationMs){
  if(delta<=0||delta>100||assist<-100||assist>100||durationMs>10000UL)return false;
  _rwProfile.cw={delta,assist,durationMs,true}; return true;
}
inline bool rwMomentumProfileSetCCW(int delta,int assist,unsigned long durationMs){
  if(delta>=0||delta<-100||assist<-100||assist>100||durationMs>10000UL)return false;
  _rwProfile.ccw={delta,assist,durationMs,true}; return true;
}
inline bool rwMomentumProfileSetRecovery(int step,unsigned long intervalMs){
  if(step<1||step>100||intervalMs<10UL||intervalMs>10000UL)return false;
  _rwProfile.recoveryStep=step; _rwProfile.recoveryIntervalMs=intervalMs; _rwProfile.recoveryConfigured=true; return true;
}
inline bool rwMomentumProfileSetTrigger(float trigger){ if(trigger<0.1f||trigger>100.0f)return false; _rwProfile.trigger=trigger; _rwProfile.triggerConfigured=true; return true; }
inline bool rwMomentumProfileReady(){
  if(!(_rwProfile.biasConfigured&&_rwProfile.cw.configured&&_rwProfile.ccw.configured&&_rwProfile.recoveryConfigured&&_rwProfile.triggerConfigured))return false;
  long cwTarget=(long)_rwProfile.bias+_rwProfile.cw.delta;
  long ccwTarget=(long)_rwProfile.bias+_rwProfile.ccw.delta;
  return cwTarget>=0&&cwTarget<=100&&ccwTarget>=0&&ccwTarget<=100;
}
inline MomentumProfile rwMomentumProfileGet(){ return _rwProfile; }
inline bool rwApplyProfileBias(){ if(!rwMomentumProfileReady())return false; return rwSetBias(_rwProfile.bias); }
inline bool rwStartProfileCW(){
  if(!rwMomentumProfileReady()||_rwState!=RW_MOMENTUM_BIAS)return false;
  _rwNominalBias=_rwProfile.bias; _rwCurrentTarget=_rwProfile.bias+_rwProfile.cw.delta; _rwDelta=_rwProfile.cw.delta; _rwAssist=_rwProfile.cw.assist; _rwDuration=_rwProfile.cw.durationMs;
  if(_rwDuration==0||_rwAssist==0){_rwDrive(_rwCurrentTarget);_rwState=RW_MOMENTUM_CONTROL_TARGET;return true;}
  _rwDrive(_rwAssist);_rwStateStart=millis();_rwState=RW_MOMENTUM_ASSIST_CW;return true;
}
inline bool rwStartProfileCCW(){
  if(!rwMomentumProfileReady()||_rwState!=RW_MOMENTUM_BIAS)return false;
  _rwNominalBias=_rwProfile.bias; _rwCurrentTarget=_rwProfile.bias+_rwProfile.ccw.delta; _rwDelta=_rwProfile.ccw.delta; _rwAssist=_rwProfile.ccw.assist; _rwDuration=_rwProfile.ccw.durationMs;
  if(_rwDuration==0||_rwAssist==0){_rwDrive(_rwCurrentTarget);_rwState=RW_MOMENTUM_CONTROL_TARGET;return true;}
  _rwDrive(_rwAssist);_rwStateStart=millis();_rwState=RW_MOMENTUM_ASSIST_CCW;return true;
}
inline bool rwStartRecovery(){
  if(!rwMomentumProfileReady())return false;
  if(_rwState==RW_MOMENTUM_ASSIST_CW||_rwState==RW_MOMENTUM_ASSIST_CCW)return false;
  _rwNominalBias=_rwProfile.bias;
  if(_rwMotorCommand==_rwNominalBias){_rwCurrentTarget=_rwNominalBias;_rwState=RW_MOMENTUM_BIAS;return true;}
  _rwRecoveryLast=millis(); _rwState=RW_MOMENTUM_RECOVERY; return true;
}
inline bool rwIsMomentumBusy(){ return _rwState==RW_MOMENTUM_ASSIST_CW||_rwState==RW_MOMENTUM_ASSIST_CCW||_rwState==RW_MOMENTUM_RECOVERY; }

inline RWMode rwGetMode(){return _rwMode;} inline RWState rwGetState(){return _rwState;}
inline int rwGetMotorCommand(){return _rwMotorCommand;} inline int rwGetCurrentBias(){return _rwNominalBias;}
inline int rwGetNominalBias(){return _rwNominalBias;} inline int rwGetDelta(){return _rwDelta;} inline int rwGetTarget(){return _rwCurrentTarget;}
inline int rwGetAssist(){return _rwAssist;} inline unsigned long rwGetDuration(){return _rwDuration;}
inline String rwModeText(){return _rwMode==RW_MODE_REACTION?"REACTION":"MOMENTUM";}
inline String rwStateText(){
  switch(_rwState){
    case RW_STOPPED:return "STOPPED"; case RW_REACTION_DRIVE:return "REACTION_DRIVE"; case RW_MOMENTUM_BIAS:return "MOMENTUM_BIAS";
    case RW_MOMENTUM_ASSIST_CW:return "MOM_ASSIST_CW"; case RW_MOMENTUM_ASSIST_CCW:return "MOM_ASSIST_CCW";
    case RW_MOMENTUM_CONTROL_TARGET:return "MOM_CONTROL_TARGET"; case RW_MOMENTUM_RECOVERY:return "MOM_RECOVERY";
  } return "UNKNOWN";
}
