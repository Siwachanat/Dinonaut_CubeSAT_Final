#pragma once

/* SunSeek Platform v2.0 — SYSTEM LAYER */
#include <Arduino.h>
#include "Module_ReactionWheel.h"
#include "System_TTC.h"
#include "System_Telemetry.h"
#include "Module_ADCS.h"
#include "Module_Estimator.h"
#include "System_TelemetryManager.h"
#include "Module_Mission.h"
#include "Module_Payload.h"
inline bool ttcParseNumber(String s,float &v){s.trim();if(!s.length())return false;char *e=nullptr;v=strtof(s.c_str(),&e);return e&&*e=='\0';}
inline String ttcNormalizeCommand(String c){c.trim();String o="";int s=0;while(s<=c.length()){int k=c.indexOf(',',s);String t=k<0?c.substring(s):c.substring(s,k);t.trim();o+=t;if(k<0)break;o+=",";s=k+1;}return o;}
inline void ttcSendRWTelemetry(){sendTelemetry("TM,RW_CMD,"+String(rwGetMotorCommand())+",RW_BIAS,"+String(rwGetNominalBias())+",RW_TARGET,"+String(rwGetTarget())+",RW_STATE,"+rwStateText());}
inline void ttcSendMomentumProfile(){MomentumProfile p=rwMomentumProfileGet();sendTelemetry("TM,MOM_PROFILE,READY,"+String(rwMomentumProfileReady()?"1":"0")+",BIAS,"+String(p.bias)+",CW_DELTA,"+String(p.cw.delta)+",CW_ASSIST,"+String(p.cw.assist)+",CW_MS,"+String(p.cw.durationMs)+",CCW_DELTA,"+String(p.ccw.delta)+",CCW_ASSIST,"+String(p.ccw.assist)+",CCW_MS,"+String(p.ccw.durationMs)+",REC_STEP,"+String(p.recoveryStep)+",REC_MS,"+String(p.recoveryIntervalMs)+",TRIGGER,"+String(p.trigger,2));}
inline void ttcSendADCSConfig(){ADCSState a=adcsGet();sendTelemetry("TM,ADCS_MODE,"+String(adcsModeText())+",ADCS_REFERENCE,"+String(adcsRefText())+",TARGET,"+String(a.target,2)+",KP,"+String(a.kp,3)+",KD,"+String(a.kd,3)+",MOMENTUM_BIAS,"+String(rwMomentumProfileReady()?rwMomentumProfileGet().bias:a.bias)+",DEADBAND,"+String(ADCS_DEADBAND_DEG,2)+",MAX_RW_COMMAND,"+String(ADCS_MAX_RW_COMMAND)+",CONTROL_SIGN,"+String(ADCS_CONTROL_SIGN,1));}
inline void ttcSendEstimatorConfig(){
  EstimatorState e=estimatorGet();
  sendTelemetry(
    "TM,ESTIMATOR,STANDARD"+
    String(",EST_FILTER,")+(e.filterEnabled?"ON":"OFF")+
    ",EST_FILTER_TYPE,"+String(estimatorFilterTypeText())+
    ",EST_MA_WINDOW,"+String(e.movingAverageWindow)+
    ",EST_FILTER_STRENGTH,"+String(e.filterStrength,2)+
    ",EST_FUSION,"+(e.fusionEnabled?"ON":"OFF")+
    ",EST_GYRO_WEIGHT,"+String(e.gyroWeight,2)
  );
}
inline void ttcStatus(){sendTelemetry("TM,SAT_ID,"+getSpacecraftID()+",BLE,"+String(isCommunicationConnected()?"CONNECTED":"DISCONNECTED")+",ADCS_MODE,"+String(adcsModeText())+",ADCS_STRATEGY,"+rwModeText());ttcSendADCSConfig();ttcSendRWTelemetry();sensorSendHealth();ttcSendEstimatorConfig();sensorSendADCSSnapshot();}
inline void ttcSafeStop(){adcsManual();rwStop();sensorSetAllStreams(false);}
inline void ttcHelp(){sendTelemetry("TM,HELP,PAYLOAD_STATUS|PAYLOAD_PING|CAPTURE|PAYLOAD_IMAGE_COUNT|PAYLOAD_LAST_IMAGE|PAYLOAD_STREAM_START|PAYLOAD_STREAM_STOP|PAYLOAD_STREAM_STATUS");sendTelemetry("TM,HELP,ADCS_MODE,MANUAL|AUTO");sendTelemetry("TM,HELP,ADCS_REFERENCE,SUN|MAG|YAW|SET_TARGET,<deg>");sendTelemetry("TM,HELP,YAW_ZERO|YAW_STATUS|YAW_RESET|YAW_TILT,ON|OFF|YAW_MAG_SIGMA,<0.5..45 deg>");sendTelemetry("TM,HELP,ADCS_STRATEGY,REACTION|MOMENTUM|ADCS_TUNE,<Kp>,<Kd>,<Bias>");sendTelemetry("TM,HELP,RW,<cmd>|RW_BIAS,<bias>|RW_CMD,<delta>,<assist>,<duration>|STATUS|STOP|SENSOR_STATUS");sendTelemetry("TM,HELP,GYRO_OFFSET|MAG_CAL_START|MAG_CAL_STOP|IMU_STATUS|CAL_STATUS|SUN_STATUS|SUN_REF_RESET");sendTelemetry("TM,HELP,MOM_PROFILE_BIAS,<bias>|MOM_PROFILE_CW,<delta>,<assist>,<ms>|MOM_PROFILE_CCW,<delta>,<assist>,<ms>|MOM_PROFILE_RECOVERY,<step>,<ms>|MOM_PROFILE_TRIGGER,<u>|MOM_PROFILE_STATUS|MOM_PROFILE_CLEAR");sendTelemetry("TM,HELP,ESTIMATOR_STATUS|ESTIMATOR_FILTER_ENABLE,ON|OFF|ESTIMATOR_FILTER_TYPE,MOVING_AVERAGE|LPF|CUSTOM|ESTIMATOR_MA_WINDOW,<1..500>|ESTIMATOR_FILTER,<0..1>|ESTIMATOR_FUSION,ON|OFF|ESTIMATOR_GYRO_WEIGHT,<0..1>");}
inline bool parse3(String p,float&a,float&b,float&c){int i=p.indexOf(','),j=p.indexOf(',',i+1);if(i<0||j<0||p.indexOf(',',j+1)>=0)return false;return ttcParseNumber(p.substring(0,i),a)&&ttcParseNumber(p.substring(i+1,j),b)&&ttcParseNumber(p.substring(j+1),c);}
inline bool parse2(String p,float&a,float&b){int i=p.indexOf(',');if(i<0||p.indexOf(',',i+1)>=0)return false;return ttcParseNumber(p.substring(0,i),a)&&ttcParseNumber(p.substring(i+1),b);}
inline bool parseTune(String p,float&kp,float&kd,float&b){int a=p.indexOf(','),c=p.indexOf(',',a+1);if(a<0||c<0||p.indexOf(',',c+1)>=0)return false;return ttcParseNumber(p.substring(0,a),kp)&&ttcParseNumber(p.substring(a+1,c),kd)&&ttcParseNumber(p.substring(c+1),b);}
inline void processTelecommand(String command){command=ttcNormalizeCommand(command);if(!command.length())return;
 if(command=="PING"){sendTelemetry("PONG");return;}
 if(command=="PAYLOAD_PING"){payloadSendCommand("PING");sendTelemetry("ACK,PAYLOAD_PING");return;}
 if(command=="PAYLOAD_STATUS"){payloadSendCommand("STATUS");sendTelemetry("ACK,PAYLOAD_STATUS");return;}
 if(command=="CAPTURE"){payloadSendCommand("CAPTURE");sendTelemetry("ACK,CAPTURE");return;}
 if(command=="PAYLOAD_IMAGE_COUNT"){payloadSendCommand("IMAGE_COUNT");sendTelemetry("ACK,PAYLOAD_IMAGE_COUNT");return;}
 if(command=="PAYLOAD_LAST_IMAGE"){payloadSendCommand("LAST_IMAGE");sendTelemetry("ACK,PAYLOAD_LAST_IMAGE");return;}
 if(command=="PAYLOAD_STREAM_START"){payloadSendCommand("STREAM_START");sendTelemetry("ACK,PAYLOAD_STREAM_START");return;}
 if(command=="PAYLOAD_STREAM_STOP"){payloadSendCommand("STREAM_STOP");sendTelemetry("ACK,PAYLOAD_STREAM_STOP");return;}
 if(command=="PAYLOAD_STREAM_STATUS"){payloadSendCommand("STREAM_STATUS");sendTelemetry("ACK,PAYLOAD_STREAM_STATUS");return;}
if(command=="STATUS"){sendTelemetry("ACK,STATUS");ttcStatus();return;} if(command=="HELP"){ttcHelp();return;} if(command=="STOP"){ttcSafeStop();sendTelemetry("ACK,STOP");sendTelemetry("EVT,SAFE");return;} if(command=="RW_STOP"){adcsManual();rwStop();sendTelemetry("ACK,RW_STOP");return;}
 if(command=="TM_STREAM,ALL,ON"){telemetryStreamSetAll(true);sendTelemetry("ACK,TM_STREAM,ALL,ON");return;}
 if(command=="TM_STREAM,ALL,OFF"){telemetryStreamSetAll(false);sendTelemetry("ACK,TM_STREAM,ALL,OFF");return;}
 if(command=="TM_STREAM,SUN,ON"){_tmStream.sun=true;sendTelemetry("ACK,TM_STREAM,SUN,ON");return;}
 if(command=="TM_STREAM,SUN,OFF"){_tmStream.sun=false;sendTelemetry("ACK,TM_STREAM,SUN,OFF");return;}
 if(command=="TM_STREAM,MAG,ON"){_tmStream.mag=true;sendTelemetry("ACK,TM_STREAM,MAG,ON");return;}
 if(command=="TM_STREAM,MAG,OFF"){_tmStream.mag=false;sendTelemetry("ACK,TM_STREAM,MAG,OFF");return;}
 if(command=="TM_STREAM,GYRO,ON"){_tmStream.gyro=true;sendTelemetry("ACK,TM_STREAM,GYRO,ON");return;}
 if(command=="TM_STREAM,GYRO,OFF"){_tmStream.gyro=false;sendTelemetry("ACK,TM_STREAM,GYRO,OFF");return;}
 if(command=="TM_STREAM,ADCS,ON"){_tmStream.adcs=true;sendTelemetry("ACK,TM_STREAM,ADCS,ON");return;}
 if(command=="TM_STREAM,ADCS,OFF"){_tmStream.adcs=false;sendTelemetry("ACK,TM_STREAM,ADCS,OFF");return;}
 if(command=="TM_STREAM,QUIET"){telemetryStreamSetAll(false);_tmStream.adcs=false;sendTelemetry("ACK,TM_STREAM,QUIET");return;}
 if(command.startsWith("TM_RATE,")){float v;if(!ttcParseNumber(command.substring(8),v)||v<1||v>20){sendTelemetry("ERR,TM_RATE_RANGE_1_TO_20");return;}telemetryStreamSetRate((uint16_t)roundf(v));sendTelemetry("ACK,TM_RATE,"+String(_tmStream.rateHz));return;}
 if(command=="TM_SNAPSHOT,ALL"){sensorSendSelectedSnapshot(true,true,true);sendTelemetry("ACK,TM_SNAPSHOT,ALL");return;}
 if(command=="TM_SNAPSHOT,SUN"){sensorSendSelectedSnapshot(true,false,false);sendTelemetry("ACK,TM_SNAPSHOT,SUN");return;}
 if(command=="TM_SNAPSHOT,MAG"){sensorSendSelectedSnapshot(false,true,false);sendTelemetry("ACK,TM_SNAPSHOT,MAG");return;}
 if(command=="TM_SNAPSHOT,GYRO"){sensorSendSelectedSnapshot(false,false,true);sendTelemetry("ACK,TM_SNAPSHOT,GYRO");return;}
 if(command=="TM_STREAM_STATUS"){sendTelemetry("TM,TM_STREAM,SUN,"+String(_tmStream.sun?"ON":"OFF")+",MAG,"+String(_tmStream.mag?"ON":"OFF")+",GYRO,"+String(_tmStream.gyro?"ON":"OFF")+",ADCS,"+String(_tmStream.adcs?"ON":"OFF")+",RATE_HZ,"+String(_tmStream.rateHz));return;}

 if(command=="MISSION_STATUS"){MissionRuntime m=missionGet();sendTelemetry("TM,MISSION_STATE,"+String(missionStateText())+",TARGET_INDEX,"+String(m.currentIndex+1)+",TARGET_COUNT,"+String(m.targetCount)+",TIME_LIMIT_REACHED,"+String(m.timeLimitReached?"1":"0")+",MISSION_ADVANCE,1,IMAGES,"+String(m.imagesCaptured));return;}
 if(command=="MISSION_CLEAR"){if(!missionClearTargets()){sendTelemetry("ERR,MISSION_CLEAR_NOT_IDLE");return;}sendTelemetry("ACK,MISSION_CLEAR");return;}
 /* [Team 2026-10-08] MISSION_TARGET,<angle>,<tolerance>,<hold>[,<capture 0|1>]. The 4th field chooses whether a picture
    is taken at this target; without it the default is MISSION_CAPTURE_ENABLE, as before. */
 if(command.startsWith("MISSION_TARGET,")){String p=command.substring(15);int a=p.indexOf(',');int b=p.indexOf(',',a+1);if(a<0||b<0){sendTelemetry("ERR,MISSION_TARGET_FORMAT");return;}
   int c=p.indexOf(',',b+1);float angle,tol,hold,cap=0;bool hasCap=(c>=0);
   if(!ttcParseNumber(p.substring(0,a),angle)||!ttcParseNumber(p.substring(a+1,b),tol)||!ttcParseNumber(hasCap?p.substring(b+1,c):p.substring(b+1),hold)){sendTelemetry("ERR,MISSION_TARGET_INVALID");return;}
   if(hasCap&&(!ttcParseNumber(p.substring(c+1),cap)||(cap!=0.0f&&cap!=1.0f))){sendTelemetry("ERR,MISSION_TARGET_INVALID");return;}
   bool ok=hasCap?missionAddTarget(angle,tol,hold,cap==1.0f):missionAddTarget(angle,tol,hold);
   if(!ok){sendTelemetry("ERR,MISSION_TARGET_INVALID");return;}
   sendTelemetry("ACK,MISSION_TARGET,"+String(_mission.targetCount)+(hasCap?String(",CAPTURE,")+(cap==1.0f?"1":"0"):String("")));return;}
 if(command.startsWith("MISSION_MAX_MIN,")){float v;if(!ttcParseNumber(command.substring(16),v)||!missionSetMaxMinutes((uint16_t)roundf(v))){sendTelemetry("ERR,MISSION_MAX_MIN_INVALID");return;}sendTelemetry("ACK,MISSION_MAX_MIN,"+String((uint16_t)roundf(v)));return;}
 if(command=="MISSION_PREPARE"){if(!missionPrepare()){sendTelemetry("ERR,MISSION_PREPARE");return;}sendTelemetry("ACK,MISSION_PREPARE");sendTelemetry("EVT,MISSION_READY");return;}
 if(command=="MISSION_START"){if(!missionStart()){sendTelemetry("ERR,MISSION_START_NOT_READY");return;}sendTelemetry("ACK,MISSION_START");return;}
 if(command=="MISSION_ABORT"){missionAbort();sendTelemetry("ACK,MISSION_ABORT");sendTelemetry("EVT,MISSION_ABORTED");return;}
 if(command=="MISSION_RESET"){missionReset();sendTelemetry("ACK,MISSION_RESET");return;}

 if(command=="ESTIMATOR_STATUS"){sendTelemetry("ACK,ESTIMATOR_STATUS");ttcSendEstimatorConfig();sensorSendEstimatorSnapshot();return;}
 if(command=="ESTIMATOR_FILTER_ENABLE,ON"){estimatorSetFilterEnabled(true);sendTelemetry("ACK,ESTIMATOR_FILTER_ENABLE,ON");ttcSendEstimatorConfig();return;}
 if(command=="ESTIMATOR_FILTER_ENABLE,OFF"){estimatorSetFilterEnabled(false);sendTelemetry("ACK,ESTIMATOR_FILTER_ENABLE,OFF");ttcSendEstimatorConfig();return;}
 if(command=="ESTIMATOR_FILTER_TYPE,MOVING_AVERAGE"){estimatorSetFilterType(EST_FILTER_MOVING_AVERAGE);sendTelemetry("ACK,ESTIMATOR_FILTER_TYPE,MOVING_AVERAGE");ttcSendEstimatorConfig();return;}
 if(command=="ESTIMATOR_FILTER_TYPE,LPF"){estimatorSetFilterType(EST_FILTER_LPF);sendTelemetry("ACK,ESTIMATOR_FILTER_TYPE,LPF");ttcSendEstimatorConfig();return;}
 if(command=="ESTIMATOR_FILTER_TYPE,CUSTOM"){estimatorSetFilterType(EST_FILTER_CUSTOM);sendTelemetry("ACK,ESTIMATOR_FILTER_TYPE,CUSTOM");ttcSendEstimatorConfig();return;}
 if(command.startsWith("ESTIMATOR_MA_WINDOW,")){float v;if(!ttcParseNumber(command.substring(20),v)||!estimatorSetMovingAverageWindow((int)roundf(v))){sendTelemetry("ERR,ESTIMATOR_MA_WINDOW_RANGE_1_TO_500");return;}sendTelemetry("ACK,ESTIMATOR_MA_WINDOW,"+String((int)roundf(v)));ttcSendEstimatorConfig();return;}
 if(command=="ESTIMATOR_LPF,ON"){estimatorSetLPFEnabled(true);sendTelemetry("ACK,ESTIMATOR_LPF,ON");ttcSendEstimatorConfig();return;}
 if(command=="ESTIMATOR_LPF,OFF"){estimatorSetLPFEnabled(false);sendTelemetry("ACK,ESTIMATOR_LPF,OFF");ttcSendEstimatorConfig();return;}
 if(command.startsWith("ESTIMATOR_FILTER,")){float v;if(!ttcParseNumber(command.substring(17),v)||!estimatorSetFilterStrength(v)){sendTelemetry("ERR,ESTIMATOR_FILTER_RANGE_0_TO_1");return;}sendTelemetry("ACK,ESTIMATOR_FILTER,"+String(v,2));ttcSendEstimatorConfig();return;}
 if(command=="ESTIMATOR_FUSION,ON"){estimatorSetFusionEnabled(true);sendTelemetry("ACK,ESTIMATOR_FUSION,ON");ttcSendEstimatorConfig();return;}
 if(command=="ESTIMATOR_FUSION,OFF"){estimatorSetFusionEnabled(false);sendTelemetry("ACK,ESTIMATOR_FUSION,OFF");ttcSendEstimatorConfig();return;}
 if(command.startsWith("ESTIMATOR_GYRO_WEIGHT,")){float v;if(!ttcParseNumber(command.substring(22),v)||!estimatorSetGyroWeight(v)){sendTelemetry("ERR,ESTIMATOR_GYRO_WEIGHT_RANGE_0_TO_1");return;}sendTelemetry("ACK,ESTIMATOR_GYRO_WEIGHT,"+String(v,2));ttcSendEstimatorConfig();return;}
 if(command=="ADCS_PREPARE"){
   ADCSState a=adcsGet();
   if(!imuGyroReady()){sendTelemetry("ERR,ADCS_PREPARE,GYRO_NOT_READY");return;}
   if(rwGetMode()==RW_MODE_MOMENTUM){
     if(!rwMomentumProfileReady()){sendTelemetry("ERR,ADCS_PREPARE,MOMENTUM_PROFILE_NOT_READY");return;}
     rwApplyProfileBias();
     sendTelemetry("TM,ADCS_PREPARE,STRATEGY,MOMENTUM,BIAS,"+String(rwGetNominalBias())+",QUALIFICATION,BASELINE");
   }else{
     sendTelemetry("TM,ADCS_PREPARE,STRATEGY,REACTION,BIAS,0");
   }
   sendTelemetry("ACK,ADCS_PREPARE");
   sendTelemetry("EVT,ADCS_READY");
   return;
 }
 if(command=="ADCS_MODE,MANUAL"){adcsManual();sendTelemetry("ACK,ADCS_MODE,MANUAL");ttcSendADCSConfig();return;} if(command=="ADCS_MODE,AUTO"){if(rwGetMode()==RW_MODE_MOMENTUM&&!rwMomentumProfileReady()){sendTelemetry("ERR,AUTO_MOMENTUM_PROFILE_NOT_READY");return;}if(!adcsAuto()){sendTelemetry("ERR,ADCS_SENSOR_NOT_READY");return;}sendTelemetry("ACK,ADCS_MODE,AUTO");ttcSendADCSConfig();return;}
 if(command=="ADCS_REFERENCE,SUN"){if(!adcsReference(ADCS_SUN)){sendTelemetry("ERR,REFERENCE_CHANGE_REQUIRES_MANUAL");return;}sendTelemetry("ACK,ADCS_REFERENCE,SUN");sensorSendADCSSnapshot();return;} if(command=="ADCS_REFERENCE,MAG"){if(!adcsReference(ADCS_MAG)){sendTelemetry("ERR,REFERENCE_CHANGE_REQUIRES_MANUAL");return;}sendTelemetry("ACK,ADCS_REFERENCE,MAG");sensorSendADCSSnapshot();return;}
 /* [Team] Relative-yaw EKF (Module_YawEKF.h, Config_YawEKF.h). ADCS_REFERENCE,YAW flies on it:
    SET_TARGET / MISSION_TARGET are then -180..+180 deg from the zero datum, + = rotated right.
    YAW_ZERO makes the present attitude the datum. It is refused only while the controller is
    flying on the EKF (AUTO + YAW), so the datum can be taken while SUN or MAG holds a target. */
 if(command=="ADCS_REFERENCE,YAW"){if(!adcsReference(ADCS_YAW)){sendTelemetry("ERR,REFERENCE_CHANGE_REQUIRES_MANUAL");return;}sendTelemetry("ACK,ADCS_REFERENCE,YAW");sensorSendADCSSnapshot();return;}
 if(command=="YAW_ZERO"){
   ADCSState a=adcsGet();
   if(a.mode==ADCS_AUTO&&a.ref==ADCS_YAW){sendTelemetry("ERR,YAW_ZERO_REFUSED_IN_AUTO_ON_YAW");return;}
   if(!yawEkfValid()){sendTelemetry("ERR,YAW_NOT_READY");return;}
   float h=yawEkfZero();
   sendTelemetry("ACK,YAW_ZERO,"+String(h,2));
   sensorSendYawSnapshot();
   return;
 }
 if(command=="YAW_STATUS"){sendTelemetry("ACK,YAW_STATUS");sensorSendYawStatus();return;}
 if(command=="YAW_RESET"){
   ADCSState a=adcsGet();
   if(a.mode==ADCS_AUTO&&a.ref==ADCS_YAW){sendTelemetry("ERR,YAW_RESET_REFUSED_IN_AUTO_ON_YAW");return;}
   yawEkfReset();sendTelemetry("ACK,YAW_RESET");return;
 }
 if(command=="YAW_TILT,ON"){if(!yawEkfSetTiltComp(true)){sendTelemetry("ERR,YAW_TILT_NEEDS_3D_MAG_CAL");return;}sendTelemetry("ACK,YAW_TILT,ON");return;}
 if(command=="YAW_TILT,OFF"){yawEkfSetTiltComp(false);sendTelemetry("ACK,YAW_TILT,OFF");return;}
 if(command.startsWith("YAW_MAG_SIGMA,")){float v;if(!ttcParseNumber(command.substring(14),v)||!yawEkfSetMagSigma(v)){sendTelemetry("ERR,YAW_MAG_SIGMA_RANGE_0.5_TO_45");return;}sendTelemetry("ACK,YAW_MAG_SIGMA,"+String(v,2));return;}
 if(command.startsWith("SET_TARGET,")){float t;if(!ttcParseNumber(command.substring(11),t)){sendTelemetry("ERR,TARGET_INVALID_VALUE");return;}if(!adcsTarget(t)){sendTelemetry("ERR,TARGET_OUT_OF_RANGE_OR_AUTO");return;}sendTelemetry("ACK,SET_TARGET,"+String(t,2));sensorSendADCSSnapshot();return;}
 if(command.startsWith("ADCS_TUNE,")){float kp,kd,b;if(!parseTune(command.substring(10),kp,kd,b)){sendTelemetry("ERR,ADCS_TUNE_SYNTAX");return;}if(!adcsTune(kp,kd,(int)roundf(b))){sendTelemetry("ERR,ADCS_TUNE_OUT_OF_RANGE");return;}sendTelemetry("ACK,ADCS_TUNE,"+String(kp,3)+","+String(kd,3)+","+String((int)roundf(b)));ttcSendADCSConfig();return;}
 if(command=="ADCS_STRATEGY,REACTION"||command=="ADCS_STRATEGY,MOMENTUM"){if(adcsGet().mode==ADCS_AUTO){sendTelemetry("ERR,STRATEGY_CHANGE_REQUIRES_MANUAL");return;}rwSetMode(command.endsWith("REACTION")?RW_MODE_REACTION:RW_MODE_MOMENTUM);sendTelemetry("ACK,"+command);ttcSendRWTelemetry();return;}
 if(command.startsWith("MOM_PROFILE_") && command!="MOM_PROFILE_STATUS" && adcsGet().mode==ADCS_AUTO){sendTelemetry("ERR,MOM_PROFILE_CHANGE_REQUIRES_MANUAL");return;}
 if(command.startsWith("MOM_PROFILE_BIAS,")){float v;if(!ttcParseNumber(command.substring(17),v)||!rwMomentumProfileSetBias((int)roundf(v))){sendTelemetry("ERR,MOM_PROFILE_BIAS_INVALID");return;}sendTelemetry("ACK,MOM_PROFILE_BIAS,"+String((int)roundf(v)));return;}
 if(command.startsWith("MOM_PROFILE_CW,")){float d,a,m;if(!parse3(command.substring(15),d,a,m)||!rwMomentumProfileSetCW((int)roundf(d),(int)roundf(a),(unsigned long)roundf(m))){sendTelemetry("ERR,MOM_PROFILE_CW_INVALID");return;}sendTelemetry("ACK,MOM_PROFILE_CW,"+String((int)roundf(d))+","+String((int)roundf(a))+","+String((unsigned long)roundf(m)));return;}
 if(command.startsWith("MOM_PROFILE_CCW,")){float d,a,m;if(!parse3(command.substring(16),d,a,m)||!rwMomentumProfileSetCCW((int)roundf(d),(int)roundf(a),(unsigned long)roundf(m))){sendTelemetry("ERR,MOM_PROFILE_CCW_INVALID");return;}sendTelemetry("ACK,MOM_PROFILE_CCW,"+String((int)roundf(d))+","+String((int)roundf(a))+","+String((unsigned long)roundf(m)));return;}
 if(command.startsWith("MOM_PROFILE_RECOVERY,")){float st,ms;if(!parse2(command.substring(21),st,ms)||!rwMomentumProfileSetRecovery((int)roundf(st),(unsigned long)roundf(ms))){sendTelemetry("ERR,MOM_PROFILE_RECOVERY_INVALID");return;}sendTelemetry("ACK,MOM_PROFILE_RECOVERY,"+String((int)roundf(st))+","+String((unsigned long)roundf(ms)));return;}
 if(command.startsWith("MOM_PROFILE_TRIGGER,")){float v;if(!ttcParseNumber(command.substring(20),v)||!rwMomentumProfileSetTrigger(v)){sendTelemetry("ERR,MOM_PROFILE_TRIGGER_INVALID");return;}sendTelemetry("ACK,MOM_PROFILE_TRIGGER,"+String(v,2));return;}
 if(command=="MOM_PROFILE_STATUS"){sendTelemetry("ACK,MOM_PROFILE_STATUS");ttcSendMomentumProfile();return;}
 if(command=="MOM_PROFILE_CLEAR"){if(adcsGet().mode==ADCS_AUTO){sendTelemetry("ERR,MOM_PROFILE_CLEAR_REQUIRES_MANUAL");return;}rwMomentumProfileClear();sendTelemetry("ACK,MOM_PROFILE_CLEAR");return;}
 if(command=="SENSOR_STATUS"){sendTelemetry("ACK,SENSOR_STATUS");sensorSendHealth();sensorSendGSSnapshot();return;} if(command=="IMU_STATUS"){sendTelemetry("ACK,IMU_STATUS");sensorSendIMUStatus();return;} if(command=="CAL_STATUS"){sendTelemetry("ACK,CAL_STATUS");sensorSendCalibrationStatus();return;} if(command=="GYRO_RAW"){sensorSendGyroOnce(true);return;} if(command=="MAG_RAW"){sensorSendMagOnce(true);return;} if(command=="SUN_RAW"){sensorSendSunOnce(true);return;} if(command=="SENSOR_STREAM,RAW"){_streamRaw=true;sensorSetAllStreams(true);sendTelemetry("ACK,SENSOR_STREAM,RAW");return;} if(command=="SENSOR_STREAM,CAL"||command=="SENSOR_STREAM,ON"){_streamRaw=false;sensorSetAllStreams(true);sendTelemetry("ACK,SENSOR_STREAM,CAL");return;} if(command=="SENSOR_STREAM,OFF"){sensorSetAllStreams(false);sendTelemetry("ACK,SENSOR_STREAM,OFF");return;}
 /* [Team] Sun-angle algorithm diagnostics (Module_SunSensor.h, SUN_ALGORITHM) */
 if(command=="SUN_STATUS"){sendTelemetry("ACK,SUN_STATUS");sensorSendSunStatus();return;}
 if(command=="SUN_REF_RESET"){sunRefReset();sendTelemetry("ACK,SUN_REF_RESET");return;}
 /* [PIO port] T03 calibration assistants: implemented in System_Telemetry.h but never routed in v2.1/v3.0 */
 if(command=="GYRO_OFFSET"){if(adcsGet().mode==ADCS_AUTO){sendTelemetry("ERR,GYRO_OFFSET_REQUIRES_MANUAL_MODE");return;}sensorGyroOffsetAssistant();return;}
 if(command=="MAG_CAL_START"){sensorMagCalStart();return;}
 if(command=="MAG_CAL_STOP"){sensorMagCalStop();return;}
 if(command.startsWith("RW,")||command.startsWith("RW_BIAS,")||command.startsWith("RW_CMD,")){if(adcsGet().mode==ADCS_AUTO){sendTelemetry("ERR,MANUAL_RW_COMMAND_REQUIRES_MANUAL_MODE");return;}}
 if(command.startsWith("RW,")){if(rwGetMode()!=RW_MODE_REACTION){sendTelemetry("ERR,RW_REQUIRES_REACTION_STRATEGY");return;}float v;if(!ttcParseNumber(command.substring(3),v)||v < -100||v > 100){sendTelemetry("ERR,RW_INVALID_OR_RANGE");return;}rwSetReactionCommand((int)roundf(v));sendTelemetry("ACK,RW,"+String(v,1));ttcSendRWTelemetry();return;}
 if(command.startsWith("RW_BIAS,")){if(rwGetMode()!=RW_MODE_MOMENTUM){sendTelemetry("ERR,RW_BIAS_REQUIRES_MOMENTUM_STRATEGY");return;}float v;if(!ttcParseNumber(command.substring(8),v)||v<0||v>100){sendTelemetry("ERR,RW_BIAS_INVALID_OR_RANGE");return;}rwSetBias((int)roundf(v));sendTelemetry("ACK,RW_BIAS,"+String(v,1));ttcSendRWTelemetry();return;}
 if(command.startsWith("RW_CMD,")){if(rwGetMode()!=RW_MODE_MOMENTUM){sendTelemetry("ERR,RW_CMD_REQUIRES_MOMENTUM_STRATEGY");return;}float d,a,m;if(!parse3(command.substring(7),d,a,m)||m<0||m>10000||!rwMomentumCommand((int)roundf(d),(int)roundf(a),(unsigned long)roundf(m))){sendTelemetry("ERR,RW_CMD_INVALID_OR_RANGE");return;}sendTelemetry("ACK,RW_CMD,"+String((int)roundf(d))+","+String((int)roundf(a))+","+String((unsigned long)roundf(m)));ttcSendRWTelemetry();return;}
 if(command=="PREPARE"||command=="START_MISSION"||command=="MISSION_STATUS"||command=="ABORT"){sendTelemetry("ERR,MISSION_NOT_AVAILABLE_T04");return;} sendTelemetry("ERR,UNKNOWN_COMMAND");}
