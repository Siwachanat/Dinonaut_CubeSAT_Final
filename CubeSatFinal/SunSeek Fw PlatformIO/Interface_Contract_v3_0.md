# SunSeek Firmware v3.0 — Momentum Profile Protocol (Draft Frozen for Integration Test)

## Existing commands retained
- `RW,<cmd>` — manual Reaction strategy, -100..100
- `RW_BIAS,<bias>` — set nominal momentum bias, 0..100
- `RW_CMD,<delta>,<assist>,<duration_ms>` — manual Momentum characterization primitive
- `ADCS_MODE,MANUAL|AUTO`
- `ADCS_STRATEGY,REACTION|MOMENTUM`
- `ADCS_TUNE,<Kp>,<Kd>,<Bias>` (Bias retained for compatibility; Momentum v3 AUTO uses profile bias)
- `ADCS_PREPARE`, `RW_STOP`, `STOP`

## v3.0 profile configuration
- `MOM_PROFILE_BIAS,<0..100>`
- `MOM_PROFILE_CW,<positive_delta>,<assist_-100..100>,<duration_ms_0..10000>`
- `MOM_PROFILE_CCW,<negative_delta>,<assist_-100..100>,<duration_ms_0..10000>`
- `MOM_PROFILE_RECOVERY,<step_1..100>,<interval_ms_10..10000>`
- `MOM_PROFILE_TRIGGER,<control_demand_0.1..100>`
- `MOM_PROFILE_STATUS`
- `MOM_PROFILE_CLEAR` (MANUAL only)

All five profile elements (Bias/CW/CCW/Recovery/Trigger) must be configured and CW/CCW targets must remain within 0..100 before Momentum AUTO/PREPARE is allowed.

## State authority
GS stores the profile. OBC holds the applied runtime profile and executes all timing-critical Assist/Recovery actions.

## AUTO Momentum
`u = Kp*error - Kd*body_rate` remains the controller. In Momentum strategy, `u` is interpreted as control demand:
- demand >= Trigger: execute CW profile
- demand <= -Trigger: execute CCW profile
- settled / below Trigger: recover toward nominal Bias

The OBC never restarts an Assist or Recovery every control-loop tick.

## RW telemetry
`TM,RW_CMD,<motor>,RW_BIAS,<nominal>,RW_TARGET,<target>,RW_STATE,<state>`

States include `MOMENTUM_BIAS`, `MOM_ASSIST_CW`, `MOM_ASSIST_CCW`, `MOM_CONTROL_TARGET`, `MOM_RECOVERY`.

## Events
- `EVT,RW_MANEUVER_COMPLETE,<target>`
- `EVT,MOM_BIAS_RECOVERED,<bias>`

## Explicit errors
- `ERR,AUTO_MOMENTUM_PROFILE_NOT_READY`
- `ERR,ADCS_PREPARE,MOMENTUM_PROFILE_NOT_READY`
- profile-field-specific INVALID errors

## Integration-test note
`ADCS_RATE_DEADBAND_DPS` is initially 2.0 deg/s and is intentionally marked as a training baseline, not a frozen hardware-derived threshold. Characterize before production freeze.
