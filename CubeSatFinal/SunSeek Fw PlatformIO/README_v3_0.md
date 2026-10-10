# SunSeek Platform Firmware v3.0

Major T04 control architecture revision based on v2.1.

## Key changes
- Profile-based Momentum AUTO: PD output is control demand, not direct bias+PWM.
- Runtime Momentum Profile: Bias, CW, CCW, Recovery, Trigger.
- Nominal Bias is separated from Current Target.
- Non-blocking Assist and Bias Recovery.
- Manual `RW_CMD,<delta>,<assist>,<duration>` parser restored/implemented for characterization.
- Explicit profile readiness errors for Momentum AUTO/PREPARE.
- Richer RW telemetry includes nominal bias, target and state.

## Safety
`STOP` and `RW_STOP` remain immediate commands. Maneuvers are non-blocking; communication and telemetry continue while Assist/Recovery executes.

## Important engineering limitation
PWM is not wheel RPM. The current platform has no wheel-speed feedback. Momentum profiles are empirical spacecraft-response profiles and must be characterized on the actual training rig.
