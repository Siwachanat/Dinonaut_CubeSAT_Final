#pragma once

/* SunSeek Platform v2.0 — MODULE LAYER */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "Config_Sensor.h"

/*
  IMU.h — T03 lightweight GY-89 driver
  No external sensor library is required.
  BMP180 is presence-checked only; pressure/temperature processing is outside
  the T03 ADCS sensing objective.
*/

#define LSM303D_ADDR       0x1D
#define LSM303D_WHO_REG    0x0F
#define LSM303D_WHO_VALUE  0x49
#define BMP180_ADDR        0x77

#if IMU_VARIANT == IMU_GY89_STANDARD
  #define GYRO_ADDR       0x6B
  #define GYRO_WHO_VALUE  0xD4
  #define GYRO_NAME       "L3GD20"
#elif IMU_VARIANT == IMU_GY89_L3G4200D
  #define GYRO_ADDR       0x69
  #define GYRO_WHO_VALUE  0xD3
  #define GYRO_NAME       "L3G4200D"
#else
  #error "Unsupported IMU_VARIANT"
#endif

#define GYRO_WHO_REG 0x0F
#define GYRO_SENSITIVITY_DPS_PER_LSB 0.00875f
#define MAG_SENSITIVITY_UT_PER_LSB 0.016f  // LSM303D +/-4 gauss: 0.160 mgauss/LSB = 0.016 uT/LSB

struct IMURawSample {
  int16_t ax, ay, az;
  int16_t mx, my, mz;
  int16_t gx, gy, gz;
  bool valid;
};

struct IMUProcessedSample {
  float gyroX, gyroY, gyroZ;
  float bodyRate;
  float magX, magY, magZ;
  float heading;
  bool valid;
};

static bool _imuLSMReady = false;
static bool _imuGyroReady = false;
static bool _imuBaroDetected = false;

inline bool imuI2CAck(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

inline bool imuWriteReg(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

inline bool imuReadReg(uint8_t addr, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return true;
}

inline bool imuReadBytes(uint8_t addr, uint8_t startReg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(addr);
  Wire.write(startReg | 0x80); // auto increment for ST sensors used here
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, length) != length) return false;
  for (uint8_t i = 0; i < length; ++i) buffer[i] = Wire.read();
  return true;
}

inline bool imuVerifyIdentity(uint8_t addr, uint8_t expected) {
  uint8_t who = 0;
  return imuI2CAck(addr) && imuReadReg(addr, 0x0F, who) && who == expected;
}

inline bool imuInitLSM303D() {
  // Accelerometer: 50 Hz, XYZ enabled, +/-2 g.
  if (!imuWriteReg(LSM303D_ADDR, 0x20, 0x57)) return false;
  if (!imuWriteReg(LSM303D_ADDR, 0x21, 0x00)) return false;
  // Magnetometer: high resolution, +/-4 gauss, continuous conversion.
  // [PIO port] CTRL5 0x64 -> 0x70: M_ODR 6.25 Hz -> 50 Hz (bits 4:2 = 100), same high resolution.
  // The ADCS loop runs at 50 Hz; at 6.25 Hz it saw a new heading only every 160 ms.
  // (100 Hz would need the accelerometer above 50 Hz, so 50 Hz is the limit here.)
  if (!imuWriteReg(LSM303D_ADDR, 0x24, 0x70)) return false;
  if (!imuWriteReg(LSM303D_ADDR, 0x25, 0x20)) return false;
  if (!imuWriteReg(LSM303D_ADDR, 0x26, 0x00)) return false;
  return true;
}

inline bool imuInitGyro() {
  // Normal mode, XYZ enabled; +/-250 dps.
  if (!imuWriteReg(GYRO_ADDR, 0x20, 0x0F)) return false;
  if (!imuWriteReg(GYRO_ADDR, 0x23, 0x00)) return false;
  return true;
}

inline void imuBegin() {
  Wire.begin(IMU_I2C_SDA, IMU_I2C_SCL);
  Wire.setClock(IMU_I2C_CLOCK_HZ);
  delay(50);

  _imuLSMReady = imuVerifyIdentity(LSM303D_ADDR, LSM303D_WHO_VALUE);
  if (_imuLSMReady) _imuLSMReady = imuInitLSM303D();

  _imuGyroReady = imuVerifyIdentity(GYRO_ADDR, GYRO_WHO_VALUE);
  if (_imuGyroReady) _imuGyroReady = imuInitGyro();

  _imuBaroDetected = imuI2CAck(BMP180_ADDR);
}

inline bool imuReadRaw(IMURawSample &s) {
  s.valid = false;
  if (!_imuLSMReady || !_imuGyroReady) return false;

  uint8_t a[6], m[6], g[6];
  if (!imuReadBytes(LSM303D_ADDR, 0x28, a, 6)) return false;
  if (!imuReadBytes(LSM303D_ADDR, 0x08, m, 6)) return false;
  if (!imuReadBytes(GYRO_ADDR, 0x28, g, 6)) return false;

  s.ax = (int16_t)(((uint16_t)a[1] << 8) | a[0]);
  s.ay = (int16_t)(((uint16_t)a[3] << 8) | a[2]);
  s.az = (int16_t)(((uint16_t)a[5] << 8) | a[4]);
  s.mx = (int16_t)(((uint16_t)m[1] << 8) | m[0]);
  s.my = (int16_t)(((uint16_t)m[3] << 8) | m[2]);
  s.mz = (int16_t)(((uint16_t)m[5] << 8) | m[4]);
  s.gx = (int16_t)(((uint16_t)g[1] << 8) | g[0]);
  s.gy = (int16_t)(((uint16_t)g[3] << 8) | g[2]);
  s.gz = (int16_t)(((uint16_t)g[5] << 8) | g[4]);
  s.valid = true;
  return true;
}

inline float imuAxisValue(float x, float y, float z, int axis) {
  if (axis == IMU_AXIS_X) return x;
  if (axis == IMU_AXIS_Y) return y;
  return z;
}

inline bool imuProcess(const IMURawSample &r, IMUProcessedSample &p) {
  p.valid = false;
  if (!r.valid) return false;

  p.gyroX = r.gx * GYRO_SENSITIVITY_DPS_PER_LSB - GYRO_BIAS_X_DPS;
  p.gyroY = r.gy * GYRO_SENSITIVITY_DPS_PER_LSB - GYRO_BIAS_Y_DPS;
  p.gyroZ = r.gz * GYRO_SENSITIVITY_DPS_PER_LSB - GYRO_BIAS_Z_DPS;
  p.bodyRate = IMU_BODY_RATE_SIGN * imuAxisValue(p.gyroX, p.gyroY, p.gyroZ, IMU_BODY_RATE_AXIS);

  p.magX = (r.mx - MAG_OFFSET_X) * MAG_SCALE_X * MAG_SENSITIVITY_UT_PER_LSB;
  p.magY = (r.my - MAG_OFFSET_Y) * MAG_SCALE_Y * MAG_SENSITIVITY_UT_PER_LSB;
  p.magZ = (r.mz - MAG_OFFSET_Z) * MAG_SCALE_Z * MAG_SENSITIVITY_UT_PER_LSB;

  float heading = atan2f(p.magY, p.magX) * 180.0f / PI;
  heading = MAG_HEADING_SIGN * heading + MAG_HEADING_OFFSET_DEG;
  while (heading < 0.0f) heading += 360.0f;
  while (heading >= 360.0f) heading -= 360.0f;
  p.heading = heading;
  p.valid = true;
  return true;
}

inline bool imuLSMReady() { return _imuLSMReady; }
inline bool imuGyroReady() { return _imuGyroReady; }
inline bool imuBaroDetected() { return _imuBaroDetected; }
inline const char* imuGyroName() { return GYRO_NAME; }
inline uint8_t imuGyroAddress() { return GYRO_ADDR; }
inline uint8_t imuGyroWhoExpected() { return GYRO_WHO_VALUE; }
