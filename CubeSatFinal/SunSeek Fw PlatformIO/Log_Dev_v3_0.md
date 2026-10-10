# v3.0 Development Notes

- Major T04 architecture revision from v2.1.
- Preserved Config/Module/System organization and existing sensor/estimator/payload/mission services.
- Reworked Momentum AUTO from direct `Bias + PD` PWM mapping to profile-based strategy execution.
- Added runtime profile and non-blocking recovery.
- Separated nominal bias from current control target.
- Added/finished RW_CMD parsing for manual momentum characterization.
- No wheel RPM sensing is assumed.
