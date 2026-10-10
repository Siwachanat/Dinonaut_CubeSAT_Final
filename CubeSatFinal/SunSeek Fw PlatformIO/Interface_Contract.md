# Interface_Contract — SunSeek Platform Firmware v2.1

This is the consolidated Ground Station ↔ Spacecraft interface reference.
The sections below preserve the existing contracts from the FIX6 development lineage.
When duplicate historical sections differ, the later T07/FIX6-compatible behavior is
the intended runtime baseline; protocol changes should be recorded in `Log_Dev.md`.



---

## Source section: `INTERFACE_CONTRACT_ESTIMATOR.md`

# T05 v0.3 — Standard Estimator Interface Contract

## Telecommands

`ESTIMATOR_STATUS`

`ESTIMATOR_LPF,ON|OFF`

`ESTIMATOR_FILTER,<strength>`
- range 0.00..1.00
- 0 = Fast
- 1 = Smooth

`ESTIMATOR_FUSION,ON|OFF`

`ESTIMATOR_GYRO_WEIGHT,<weight>`
- range 0.00..1.00
- 0 = Reference
- 1 = Gyro

## Telemetry

Configuration:
`TM,ESTIMATOR,STANDARD,EST_LPF,<ON|OFF>,EST_FILTER_STRENGTH,<value>,EST_FUSION,<ON|OFF>,EST_GYRO_WEIGHT,<value>`

Estimator values are emitted with operational ADCS telemetry:
`TM,EST_RAW,<deg>,EST_FILTERED,<deg>,EST_ANGLE,<deg>,EST_LPF,<ON|OFF>,EST_FILTER_STRENGTH,<value>`

`TM,EST_FUSION,<ON|OFF>,EST_GYRO_WEIGHT,<value>,EST_VALID,<0|1>`

## Controller contract

The PD controller consumes `EST_ANGLE`, not the raw selected reference.

With LPF OFF and Fusion OFF:
`EST_ANGLE == selected raw reference` (apart from normal sampling timing), so
the previous T05 v0.2 control path is preserved.

The D term continues to use calibrated spacecraft Body Rate directly.


---

## Source section: `INTERFACE_CONTRACT_T04.md`

# T04 Interface Contract v0.2

## Generic control interface
`POINTING_ERROR = active reference error`
`u = Kp * POINTING_ERROR - Kd * GYRO_Z`

SUN: `error = target - sun_angle`
MAG: `error = wrap180(target - heading)`

## Commands
`ADCS_MODE,MANUAL|AUTO`
`ADCS_REFERENCE,SUN|MAG`
`SET_TARGET,<deg>`
`ADCS_STRATEGY,REACTION|MOMENTUM`
`ADCS_TUNE,<Kp>,<Kd>,<MomentumBias>`
`STOP`

Reference/target/strategy changes require MANUAL. Direct RW commands are blocked
in AUTO.

## Telemetry
`TM,ADCS_MODE,AUTO,ADCS_REFERENCE,SUN,TARGET,0.00,POINTING_ERROR,-4.20,SUN_ERROR,-4.20,MAG_ERROR,31.50,GYRO_Z,2.130,RW_CMD,-9`

All operational telemetry remains flat key/value pairs.

## Safety
Sensor read failure in AUTO stops the wheel and returns the controller to MANUAL.


---

## Source section: `INTERFACE_CONTRACT_T05.md`

# T05 Interface Contract — v0.1

## Physical UART

SunSeek ESP32-S3 uses HardwareSerial1:

- TX = GPIO41 -> ESP32-CAM RX0/GPIO3
- RX = GPIO42 <- ESP32-CAM TX0/GPIO1
- Baud = 115200, 8N1
- Common GND required

## Ground Station -> OBC -> Payload mapping

- `PAYLOAD_PING` -> UART `PING`
- `PAYLOAD_STATUS` -> UART `STATUS`
- `CAPTURE` -> UART `CAPTURE`
- `PAYLOAD_IMAGE_COUNT` -> UART `IMAGE_COUNT`
- `PAYLOAD_LAST_IMAGE` -> UART `LAST_IMAGE`

The OBC immediately ACKs accepted Ground Station commands. Payload responses arrive asynchronously.

## Payload -> OBC -> Ground Station

Payload-native UART lines are preserved with a `PAYLOAD,` prefix for engineering visibility.

Examples:

`PAYLOAD,STATUS,READY,CAMERA,OK,SD,OK,WIFI,READY,IP,192.168.4.1,IMAGE_COUNT,12,LAST_IMAGE,/IMG_0012.JPG`

`PAYLOAD,EVENT,CAPTURE_STARTED`

`PAYLOAD,IMAGE_READY,/IMG_0013.JPG,245731`

The OBC additionally emits flat `TM` fields compatible with Ground Station state parsing, including `CAMERA`, `PAYLOAD_WIFI`, `PAYLOAD_IP`, `IMAGE_COUNT`, `LAST_IMAGE`, and `IMAGE_SIZE` where available.

## Data-plane rule

JPEG bytes are not carried over BLE or UART. Ground Station retrieves images from the ESP32-CAM using Wi-Fi/HTTP.


---

## Source section: `INTERFACE_CONTRACT_T07_v0_1.md`

# T07 v0.1 Interface Contract

## Engineering telemetry
Continuous sensor TM is OFF after boot.

`TM_STREAM,<SUN|MAG|GYRO|ALL>,<ON|OFF>`
`TM_RATE,<1..20>`
`TM_STREAM_STATUS`
`TM_SNAPSHOT,<SUN|MAG|GYRO|ALL>`

This controls downlink only. It does not power sensors off and does not prevent
ADCS/Estimator from acquiring the sensors they require.

## Mission target sequence
Clear first, then add 1..10 targets:

`MISSION_CLEAR`
`MISSION_TARGET,<angleDeg>,<toleranceDeg>,<holdSec>`

Set maximum duration:
`MISSION_MAX_MIN,<1..15|20|30>`

Run:
`MISSION_PREPARE`
`MISSION_START`
`MISSION_STATUS`
`MISSION_ABORT`
`MISSION_RESET`

Maximum Mission Duration is a guard condition, not a speed score. At expiry the
firmware enters TIME_LIMIT_REACHED but does not automatically abort or RW STOP.


---

## v2.1 Actuator Command Ownership

`RW_CMD,<...>` authorization is based on spacecraft operating mode, not on a training-module name.

- MANUAL: direct `RW_CMD` permitted.
- AUTO: direct `RW_CMD` rejected with `ERR,RW_CMD_REQUIRES_MANUAL_MODE`.
- `RW_BIAS` remains available for momentum configuration/experiments according to the command router.
- `RW_STOP` remains the explicit stop/safety command.

Training intent: T01/T02 Engineering experiments may directly command the reaction wheel
while MANUAL. AUTO/mission operation delegates actuator ownership to ADCS.
