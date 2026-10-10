#pragma once
/*
  SunSeek Platform v2.0
  TRAINING LEVEL: INTERMEDIATE
  Learner-adjustable ADCS gains, limits and control timing.
*/
#define ADCS_DEFAULT_KP 2.0f
#define ADCS_DEFAULT_KD 0.5f
#define ADCS_DEFAULT_TARGET_DEG 0.0f
#define ADCS_DEADBAND_DEG 2.0f
#define ADCS_MAX_RW_COMMAND 80
#define ADCS_CONTROL_SIGN -1.0f  // [Team 2026-10-07] was +1. Positive wheel command turns the body LEFT;
                                 // angles are now right-positive (SIGN CONVENTION in Config_Sensor.h)
#define ADCS_CONTROL_PERIOD_MS 20UL
#define ADCS_TM_PERIOD_MS 250UL
#define ADCS_RATE_DEADBAND_DPS 2.0f  // v3.0 initial training baseline; characterize before freeze
#define ADCS_KP_MAX 20.0f
#define ADCS_KD_MAX 20.0f
