#pragma once

/*
  SunSeek Platform v2.0
  TRAINING LEVEL: INTERMEDIATE / ADVANCED
  Parameters may be tuned in Engineering activities.
  Algorithm implementation belongs in Module_Estimator.h.
*/

/*
  T07 FIX4 — Estimator filter configuration

  Teaching/default filter:
    MOVING_AVERAGE, 10 samples

  Advanced alternatives:
    LPF    : strength 0..1 (0 Fast, 1 Smooth)
    CUSTOM : reserved hook; currently pass-through reference until a team
             implementation is added.

  Gyro fusion remains the complementary-filter stage after the selected
  reference filter.
*/
#define ESTIMATOR_DEFAULT_FILTER_ENABLED true
#define ESTIMATOR_DEFAULT_FILTER_TYPE 1
#define ESTIMATOR_DEFAULT_MA_WINDOW 10
#define ESTIMATOR_MAX_MA_WINDOW 500

#define ESTIMATOR_DEFAULT_FILTER_STRENGTH 0.35f
#define ESTIMATOR_DEFAULT_FUSION_ENABLED true
#define ESTIMATOR_DEFAULT_GYRO_WEIGHT 0.98f

#define ESTIMATOR_MIN_DT_S 0.001f
#define ESTIMATOR_MAX_DT_S 0.250f
