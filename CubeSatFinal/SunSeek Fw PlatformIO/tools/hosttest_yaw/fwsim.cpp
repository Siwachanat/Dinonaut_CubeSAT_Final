// The WHOLE firmware (src/main.cpp + every Module_/System_ header + your Config_*.h) running on a PC
// against a simulated spacecraft: body on one axis, reaction wheel, GY-89 on a fake I2C bus.
//
// stdin  : a script, one line each:   <time_ms> <telecommand>      e.g.  2000 ADCS_REFERENCE,YAW
//          or a simulator setting:    <time_ms> @<key> <value>     keys: see simSet() below
//                                     (@camera 1 connects a fake ESP32-CAM that answers CAPTURE after 0.6 s)
//          last line:                 <time_ms> @end
// stdout : "ST <ms> <true yaw> <true rate> <wheel cmd> <ADCS est> <ADCS error> <EKF rel> <EKF abs> <mag heading> <true heading>"
//          every 20 ms, and "TM <ms> <line>" for everything the firmware sends.
//
// The spacecraft below is a MODEL. Its three sign facts come from this team's board notes
// (Log_Dev.md entry 6): gyro Z reads negative turning right, atan2(magY, magX) rises turning
// right, a positive wheel command turns the body left. Everything else is an assumption.
#include <random>
#include <deque>
#include <string>
#include <Arduino.h>      // tools/hosttest_yaw/fake/ (build with -I fake first)
#include <Wire.h>

// ------------------------------------------------------------------ simulated world
static double g_us = 0;                 // simulated time [us]
static double g_psi = 0, g_om = 0;      // true yaw [deg] and rate [deg/s], + = rotated right
static double g_wheel = 0;              // wheel speed, -1..+1 of full speed
static int g_pwm = 0, g_in1 = 0, g_in2 = 0;
static double P_heading0 = 110.0;       // heading of body X at yaw 0
static double P_kw = 300.0;             // body rate per unit wheel speed [deg/s] (momentum ratio)
static double P_tau_m = 0.5;            // wheel speed time constant [s]
static double P_tau_f = 4.0;            // rig drag time constant [s]
static double P_gyro_resid = 0.30;      // raw gyro Z bias beyond GYRO_BIAS_Z_DPS [deg/s]
static double P_gyro_noise = 0.10;      // [deg/s]
static double P_mag_noise = 0.4;        // calibrated units per axis
static double P_field = 30.0;           // horizontal field strength in calibrated units
static double P_motor = 1.0;            // motor stray field scale (x (1.6, 1.1) units at stall)
static double P_ext_x = 0, P_ext_y = 0; // external field step
static std::mt19937 g_rng(1);
static std::normal_distribution<double> g_n(0.0, 1.0);
static int16_t g_mag[3] = {0, 0, 0}; static double g_magNextUs = 3000;
// fake ESP32-CAM on UART1 (off unless "@camera 1"): CAPTURE -> IMAGE_READY,<name>,<bytes> 0.6 s later
static bool g_camOn = false; static int g_camImages = 0; static double g_camReplyUs = -1;
static std::string g_camTxLine; static std::deque<char> g_camRx;

static int motorCmd() { int p = (int)lround(g_pwm * 100.0 / 255.0); return g_in1 && !g_in2 ? p : (!g_in1 && g_in2 ? -p : 0); }

#include "Config_Sensor.h"
static void sampleMag() {
  const double a = (g_psi + P_heading0) * M_PI / 180.0;
  const double current = motorCmd() / 100.0 - g_wheel;          // drive minus back-EMF
  double c[3] = { P_field * cos(a) + 1.6 * P_motor * current + P_ext_x + P_mag_noise * g_n(g_rng),
                  P_field * sin(a) + 1.1 * P_motor * current + P_ext_y + P_mag_noise * g_n(g_rng),
                  -12.0 + P_mag_noise * g_n(g_rng) };
  // invert the firmware's calibration so that imuProcess() gives back the values above
  const double off[3] = { MAG_OFFSET_X, MAG_OFFSET_Y, MAG_OFFSET_Z }, sc[3] = { MAG_SCALE_X, MAG_SCALE_Y, MAG_SCALE_Z };
  for (int i = 0; i < 3; i++) g_mag[i] = (int16_t)lround(c[i] / (sc[i] * 0.016) + off[i]);
}
static void advance(double us) {
  while (us > 0) {
    const double h = us > 1000 ? 1000 : us, dt = h * 1e-6;
    const double dw = (motorCmd() / 100.0 - g_wheel) / P_tau_m * dt;
    g_wheel += dw;
    g_om += -P_kw * dw;                     // positive wheel command turns the body left
    g_om -= g_om * dt / P_tau_f;
    g_psi += g_om * dt;
    g_us += h; us -= h;
    if (g_us >= g_magNextUs) { sampleMag(); g_magNextUs += 20000; }   // magnetometer: its own 50 Hz clock
    if (g_camReplyUs >= 0 && g_us >= g_camReplyUs) {
      g_camReplyUs = -1; char b[64]; snprintf(b, sizeof b, "IMAGE_READY,/IMG_%04d.JPG,%d\n", ++g_camImages, 40000 + 137 * g_camImages);
      for (const char *q = b; *q; ++q) g_camRx.push_back(*q);
    }
  }
}
static void simSet(const std::string &k, double v) {
  if (k == "heading0") P_heading0 = v; else if (k == "kw") P_kw = v; else if (k == "tau_m") P_tau_m = v;
  else if (k == "tau_f") P_tau_f = v; else if (k == "gyro_resid") P_gyro_resid = v; else if (k == "gyro_noise") P_gyro_noise = v;
  else if (k == "mag_noise") P_mag_noise = v; else if (k == "motor") P_motor = v; else if (k == "ext_x") P_ext_x = v;
  else if (k == "ext_y") P_ext_y = v; else if (k == "kick") g_om += v; else if (k == "seed") g_rng.seed((unsigned)v);
  else if (k == "yaw") g_psi = v;
  else if (k == "camera") g_camOn = (v != 0);
}

// ------------------------------------------------------------------ Arduino core
unsigned long millis() { return (unsigned long)(g_us / 1000.0); }
unsigned long micros() { return (unsigned long)g_us; }
void delay(unsigned long ms) { advance(ms * 1000.0); }
void pinMode(int, int) {}
#include "Config_Actuator.h"
void digitalWrite(int pin, int level) { if (pin == RW_PIN_AIN1) g_in1 = level; if (pin == RW_PIN_AIN2) g_in2 = level; }
void analogWrite(int pin, int value) { if (pin == RW_PIN_PWMA) g_pwm = value; }
int analogRead(int pin) { advance(60); return (pin == SUN_LEFT_PIN ? 3597 : 3753) + (int)lround(4 * g_n(g_rng)); }
uint32_t analogReadMilliVolts(int pin) { return (uint32_t)(analogRead(pin) * 3100L / 4095L); }
HardwareSerial Serial(0);
static std::deque<char> g_rx;
int simSerialAvailable() { return (int)g_rx.size(); }
int simSerialRead() { if (g_rx.empty()) return -1; char c = g_rx.front(); g_rx.pop_front(); return (unsigned char)c; }
void simSerialOut(const char *line) { printf("TM %lu %s\n", millis(), line); }
void simPayloadTx(const char *text) {
  for (const char *q = text; *q; ++q) {
    if (*q == '\n') { if (g_camOn && g_camTxLine == "CAPTURE") g_camReplyUs = g_us + 600000.0; g_camTxLine.clear(); }
    else g_camTxLine.push_back(*q);
  }
}
int simPayloadAvailable() { return (int)g_camRx.size(); }
int simPayloadRead() { if (g_camRx.empty()) return -1; char c = g_camRx.front(); g_camRx.pop_front(); return (unsigned char)c; }

// ------------------------------------------------------------------ GY-89 on the I2C bus
TwoWire Wire;
static uint8_t w_addr = 0, w_reg = 0, w_n = 0, w_buf[8], w_pos = 0, w_len = 0;
void TwoWire::beginTransmission(uint8_t addr) { w_addr = addr; w_n = 0; }
size_t TwoWire::write(uint8_t b) { if (w_n == 0) w_reg = b & 0x7F; w_n++; return 1; }
uint8_t TwoWire::endTransmission(bool) { advance(250); return (w_addr == 0x1D || w_addr == 0x6B || w_addr == 0x77) ? 0 : 2; }
static void put16(uint8_t *p, double v) { if (v > 32767) v = 32767; if (v < -32768) v = -32768; int16_t x = (int16_t)lround(v); p[0] = (uint8_t)(x & 0xFF); p[1] = (uint8_t)((x >> 8) & 0xFF); }
uint8_t TwoWire::requestFrom(uint8_t addr, uint8_t n) {
  advance(100.0 + 95.0 * n);                                   // 100 kHz bus: about 0.1 ms per byte
  memset(w_buf, 0, sizeof w_buf); w_pos = 0; w_len = n > 8 ? 8 : n;
  if (addr == 0x1D && w_reg == 0x0F) w_buf[0] = 0x49;
  else if (addr == 0x6B && w_reg == 0x0F) w_buf[0] = 0xD4;
  else if (addr == 0x1D && w_reg == 0x28) { put16(w_buf, 300 * g_n(g_rng)); put16(w_buf + 2, 300 * g_n(g_rng)); put16(w_buf + 4, 16393 + 300 * g_n(g_rng)); }
  else if (addr == 0x1D && w_reg == 0x08) { memcpy(w_buf, g_mag, 6); }
  else if (addr == 0x6B && w_reg == 0x28) {
    const double k = 1.0 / 0.00875;
    put16(w_buf,     (-7.33 + 0.20 + P_gyro_noise * g_n(g_rng)) * k);
    put16(w_buf + 2, (-0.74 - 0.10 + P_gyro_noise * g_n(g_rng)) * k);
    put16(w_buf + 4, (-g_om + GYRO_BIAS_Z_DPS + P_gyro_resid + P_gyro_noise * g_n(g_rng)) * k);   // negative turning right
  }
  return w_len;
}
int TwoWire::read() { return w_pos < w_len ? w_buf[w_pos++] : -1; }

// ------------------------------------------------------------------ the firmware itself
#include "main.cpp"

int main() {
  struct Ev { double ms; std::string text; };
  std::deque<Ev> script; double endMs = 0; char line[300];
  while (fgets(line, sizeof line, stdin)) {
    char *sp = strchr(line, ' '); if (!sp) continue;
    std::string text(sp + 1); while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) text.pop_back();
    if (text == "@end") { endMs = atof(line); continue; }
    script.push_back({atof(line), text});
  }
  sampleMag();
  // settings at time 0 apply before boot
  while (!script.empty() && script.front().ms <= 0 && script.front().text[0] == '@') {
    char k[40]; double v = 0; if (sscanf(script.front().text.c_str(), "@%39s %lf", k, &v) >= 1) simSet(k, v);
    script.pop_front();
  }
  sampleMag();
  setup();
  double nextLog = 0;
  while (g_us * 1e-3 < endMs) {
    while (!script.empty() && script.front().ms <= g_us * 1e-3) {
      const std::string &t = script.front().text;
      if (t[0] == '@') { char k[40]; double v = 0; if (sscanf(t.c_str(), "@%39s %lf", k, &v) >= 1) simSet(k, v); }
      else { for (char c : t) g_rx.push_back(c); g_rx.push_back('\n'); }
      script.pop_front();
    }
    loop();
    advance(150);                                              // the rest of one pass through loop()
    if (g_us * 1e-3 >= nextLog) {
      nextLog += 20.0;
      ADCSState a = adcsGet();
      double th = fmod(g_psi + P_heading0, 360.0); if (th < 0) th += 360.0;
#ifdef FWSIM_NO_YAW        // firmware from before the yaw EKF (regression comparison)
      printf("ST %.1f %.4f %.4f %d %.4f %.4f 0 0 0 %.4f 0 0 0\n", g_us * 1e-3, g_psi, g_om, rwGetMotorCommand(),
             (double)a.estimatedAngle, (double)a.error, th);
#else
      YawEkfOut y = yawEkfGet(); YawEkfStatus s = yawEkfStatus();
      printf("ST %.1f %.4f %.4f %d %.4f %.4f %.4f %.4f %.4f %.4f %.4f %lu %lu\n", g_us * 1e-3, g_psi, g_om, rwGetMotorCommand(),
             (double)a.estimatedAngle, (double)a.error, (double)y.relDeg, (double)y.absDeg, (double)y.magDeg, th,
             (double)y.biasDps, (unsigned long)s.rejected, (unsigned long)s.reacquired);
#endif
    }
  }
  return 0;
}
