# SunSeek Platform Firmware v2.1

## Training code architecture

| Prefix | Role | Normal learner access |
|---|---|---|
| `Config_` | Parameters, calibration, gains, identity and interface settings | Edit when instructed |
| `Module_` | Subsystem / algorithm implementation | Advanced exercises |
| `System_` | TT&C, command routing and telemetry infrastructure | Do not modify in normal training |

## Runtime baseline
v2.0 reorganizes the verified source lineage of T07 FIX6 Estimator Continuous + MA500.
The reorganization is intended to preserve runtime behavior while making the firmware
easier to teach, maintain and extend.

## Main execution flow
`System_TTC` receives TC → `System_CommandRouter` dispatches → `Module_*` executes →
`System_Telemetry` / events report state to the Ground Station.

Sensor acquisition and estimator update remain independent from whether continuous
telemetry is enabled.


## v2.1 actuator ownership rule

- `MANUAL`: the operator / Engineering console owns the actuator. `RW_BIAS`,
  `RW_CMD`, and `RW_STOP` may be used for T01/T02 characterization and manual experiments.
- `AUTO`: ADCS owns the actuator. Direct `RW_CMD` is rejected; use the ADCS
  configuration / `ADCS_PREPARE` / mission flow instead.
- `RW_STOP` remains the explicit stop/safety command.

This replaces the legacy lesson-name restriction on `RW_CMD` with an operating-state rule.
