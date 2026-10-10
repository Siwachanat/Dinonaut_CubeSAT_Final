# SunSeek Platform Firmware v3.0 — PlatformIO (VS Code)

Port of the organisers' `SunSeek_Platform_Firmware_v3_0` for the competition kit
(GY-89, TB6612FNG flywheel, paired LDRs, ESP32-CAM). 15 of 17 headers are byte-identical
to the organisers' code; the two small changes are in `Log_Dev.md` → *v3.0-pio*.
Organisers' notes: `README_v3_0.md`, `Interface_Contract_v3_0.md`.

## Open, build, flash
1. VS Code → PlatformIO → **Open Project** → this folder.
2. ✓ Build → Upload → Monitor (115200). Pick `env:sunseek` (core 3.3) or, if the
   toolchain will not install, `env:sunseek_core2`.
3. Upload will not connect: hold **BOOT**, tap **RESET**, release BOOT.

## What changed from v2.1 (only Momentum and RW_CMD)
Sensors, estimator, mission, payload and BLE are identical to v2.1. REACTION AUTO is
unchanged. MOMENTUM AUTO is new:

| | v2.1 | v3.0 |
|---|---|---|
| Momentum AUTO output | `RW = bias + Kp·e − Kd·rate`, continuous | discrete **kicks** from a profile |
| Bias source | `ADCS_TUNE` 3rd value | `MOM_PROFILE_BIAS` (ADCS_TUNE bias ignored) |
| AUTO allowed when | always | only when all 5 profile parts are set |
| `RW_CMD` | not implemented | `RW_CMD,<delta>,<assist>,<ms>` (MOMENTUM, MANUAL) |
| Telemetry | `RW_CMD, RW_BIAS, RW_STATE` | + `RW_TARGET`, `TM,MOM_PROFILE,…`, `EVT,MOM_BIAS_RECOVERED` |

### How a Momentum kick works (Module_ADCS.h + Module_ReactionWheel.h)
Every 20 ms in AUTO: `demand = SIGN·(Kp·e − Kd·rate)` (0 inside the ±2° deadband).
- wheel busy (assist or recovery running) → do nothing
- `|demand| < TRIGGER`, or settled (|e| < 2° and |rate| < 2 °/s) → stay at / recover to BIAS
- `demand ≥ TRIGGER` → **CW kick**: drive `CW_ASSIST` for `CW_MS`, then hold `BIAS+CW_DELTA`
- `demand ≤ −TRIGGER` → **CCW kick** (same with CCW values)
- one cycle later the wheel ramps back to BIAS by `REC_STEP` every `REC_MS`

Speeding the wheel up gives the body a rate; ramping it back takes that rate away again.
So **each kick moves the spacecraft by a roughly fixed angle and then stops it**. That
angle depends on delta, assist, the ramp speed and the rig's friction, and has to be
measured. The console's kick log does this.

Consequences worth knowing:
- **Effective angle deadband ≈ max(2°, TRIGGER / Kp)** at rest. With Kp 2 and trigger 10
  the spacecraft will not kick until it is 5° off.
- If one kick moves the spacecraft further than the window it has to land in, it overshoots,
  kicks back, and repeats forever (a limit cycle). Make the angle per kick smaller than the
  mission tolerance.
- Cycle time ≈ `MS + 20 ms + ceil(|DELTA| / REC_STEP) × REC_MS`. No new kick can start
  until the wheel is back at BIAS.
- On a rig with friction the body slows during the hold, so the ramp-back pushes it
  *backwards*. A slow ramp on a high-friction rig can give a small or even reversed angle.

### Before Momentum AUTO (or a mission in MOMENTUM)
1. `ADCS_STRATEGY,MOMENTUM` (MANUAL).
2. Send all five profile parts (MANUAL only): `MOM_PROFILE_BIAS,b`,
   `MOM_PROFILE_CW,+d,assist,ms`, `MOM_PROFILE_CCW,−d,assist,ms`,
   `MOM_PROFILE_RECOVERY,step,ms`, `MOM_PROFILE_TRIGGER,u`. Check: `MOM_PROFILE_STATUS`
   → `READY,1`. Rules: CW delta > 0, CCW delta < 0, `BIAS+CW` ≤ 100, `BIAS+CCW` ≥ 0.
3. **The profile lives in RAM.** A reset or power cycle clears it, and AUTO is then refused
   with `ERR,AUTO_MOMENTUM_PROFILE_NOT_READY`. The console keeps a copy and re-sends it.
4. `ADCS_PREPARE` spins the wheel up to BIAS. Wait until the body stops turning, *then* go
   AUTO or start the mission. If the wheel is stopped when AUTO starts, the firmware jumps
   it straight to BIAS, and that spin-up kicks the spacecraft. `MISSION_RESET` stops the
   wheel, so prepare again after it.

## Relative yaw: `ADCS_REFERENCE,YAW` (team, 8 Oct)
A third reference next to SUN and MAG. The angle comes from a 2-state EKF (gyro Z +
magnetometer, gyro bias estimated on line) in `src/Module_YawEKF.h`; settings in
`include/Config_YawEKF.h`. SUN and MAG behave exactly as before.

| | |
|---|---|
| Angle | relative yaw, −180…+180°, **+ = rotated right** of the zero datum |
| Zero datum | boot attitude until `YAW_ZERO` (allowed any time except AUTO on YAW) |
| Targets | `SET_TARGET` / `MISSION_TARGET` −180…+180°, short way round |
| Watch | `EST_RAW` (magnetometer) against `EST_ANGLE` (EKF) in the ADCS stream; `YAW_STATUS` for health |

Typical use: `ADCS_MODE,MANUAL` → `ADCS_REFERENCE,YAW` → point the spacecraft where 0 should be
→ `YAW_ZERO` → `SET_TARGET,<deg>` → `ADCS_MODE,AUTO`. Or hold the lamp in SUN AUTO, send
`YAW_ZERO` there, then switch to YAW: every target is then an angle from the lamp, also beyond
the ±70° the sun sensor can see.

Before the first AUTO on YAW: magnetometer calibrated on the assembled stack; turn right by hand
and check `YAW_REL` rises smoothly while `YAW_REJECTED` in `YAW_STATUS` does not climb (climbing =
gyro and magnetometer signs disagree). PC checks: `python3 tools/hosttest_yaw/run.py` and
`python3 tools/hosttest_yaw/run_loop.py`.

## Inherited from v2.1 (unchanged)
- Mission stops at `CAPTURING` after target 1 (no capture, no advance).
- The mission only pushes its target if ADCS is in MANUAL at start; send `ADCS_MODE,MANUAL`
  before `MISSION_START`.
- LDRs on GPIO16/17 are ADC2: no Wi-Fi on the S3.
