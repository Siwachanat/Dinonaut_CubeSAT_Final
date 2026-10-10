#pragma once

/*
  Config_YawEKF.h — [Team 2026-10-08] relative-yaw estimator settings
  TRAINING LEVEL: INTERMEDIATE / ADVANCED

  Used by src/Module_YawEKF.h: a 2-state Extended Kalman Filter
      state        x = [ yaw , gyro bias ]
      prediction   gyroscope Z (every control cycle, 50 Hz)
      correction   magnetometer field vector (when a new sample is available)
      accelerometer: gravity direction -> tilt angle, optional tilt compensation
  Selected with the telecommand  ADCS_REFERENCE,YAW  and zeroed with  YAW_ZERO.
  The SUN and MAG references do not use anything in this file.

  The estimator trusts each sensor according to the numbers below. They are
  1-sigma values in engineering units, so they can be checked against telemetry:
  YAW_STATUS reports the measured magnetometer residual (YAW_INNOV_RMS).
*/

// ---------------- sensor trust (the tuning that matters) -------------------
// Magnetometer heading error, 1 sigma [deg]. Includes noise, motor field and
// calibration residue. Larger = smoother estimate that leans on the gyro for
// longer; smaller = follows the magnetometer faster. Runtime: YAW_MAG_SIGMA,<deg>
#define YAW_EKF_MAG_SIGMA_DEG      3.0f

// Gyro angle random walk [deg/sqrt(s)]. 0.07 = 0.5 deg/s of white rate noise at 50 Hz.
#define YAW_EKF_GYRO_ARW           0.07f

// How fast the gyro bias is allowed to wander [deg/s per sqrt(s)].
#define YAW_EKF_BIAS_RW            0.01f

// Gyro scale-factor uncertainty (0.02 = 2 %). Adds uncertainty while turning fast,
// so the magnetometer is weighted more just after a large slew.
#define YAW_EKF_GYRO_SCALE_UNC     0.02f

// ---------------- gyro bias state -----------------------------------------
#define YAW_EKF_INIT_BIAS_SIGMA    1.0f    // [deg/s] uncertainty of GYRO_BIAS_Z_DPS at start
#define YAW_EKF_BIAS_MAX_DPS       5.0f    // [deg/s] estimate is clamped to +/- this
// The bias is only learnt while the body turns slower than this. While turning,
// a magnetometer calibration error looks exactly like a rate error.
#define YAW_EKF_BIAS_LEARN_MAX_DPS 6.0f

// ---------------- magnetometer sample checks ------------------------------
// A sample whose residual is larger than this many sigma is not used.
#define YAW_EKF_GATE_SIGMA         3.0f
// Residuals outside the gate for this long [s] mean the estimate is wrong, not the
// sample: the filter then re-acquires the heading from the magnetometer.
#define YAW_EKF_REACQUIRE_S        1.0f
// Field strength is tracked with this time constant [s] ...
#define YAW_EKF_FIELD_TAU_S        3.0f
// ... and a sample this far (fraction) from it has its variance doubled.
#define YAW_EKF_FIELD_TOL          0.15f
// Rotation rate about the other two axes (swinging, being handled) [deg/s] at
// which the magnetometer variance is doubled.
#define YAW_EKF_SWING_RATE_DPS     8.0f

// ---------------- accelerometer (tilt) ------------------------------------
// 1 = project the magnetic field onto the horizontal plane found from the
// accelerometer before taking the heading (tilt compensation). Runtime: YAW_TILT,ON|OFF
// Needs a magnetometer calibration made by tumbling the stack through ALL
// orientations: MAG_SCALE_Z must come out between 0.5 and 2.0. The firmware
// refuses tilt compensation otherwise (YAW_STATUS shows TILT_COMP,REFUSED).
#define YAW_EKF_TILT_COMP          0
#define YAW_EKF_ACC_TAU_S          1.0f     // gravity low-pass [s]; removes wheel vibration
#define YAW_EKF_ACC_1G_COUNTS      16393.0f // LSM303D +/-2 g: 0.061 mg/LSB
#define YAW_EKF_ACC_NORM_TOL       0.15f    // |a| must be within 15 % of 1 g to be used

// ---------------- timing ---------------------------------------------------
// A gap longer than this [s] between two cycles (e.g. GYRO_OFFSET blocks for 3 s)
// is not integrated; the heading is re-acquired from the magnetometer instead.
#define YAW_EKF_MAX_DT_S           0.25f
