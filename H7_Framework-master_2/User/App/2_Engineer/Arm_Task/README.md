# Engineer Arm Control

## Control modes

`Arm_Control_Config` is the only runtime tuning entry.

- `master_enable = 0`: all six joints use position-speed mode.
- `axis_mode[n] = 0`: position-speed mode.
- `axis_mode[n] = 1`: gravity compensation.
- `axis_mode[n] = 2`: gravity compensation plus external impedance.

Only axes 1, 3, and 4 (J2, J4, and J5) accept gravity or impedance modes.
The defaults are safe: `master_enable` is zero and every axis is in position mode.

Recommended first test:

1. Support the arm mechanically and confirm all feedback is online.
2. Set `master_enable` to 1.
3. Set `axis_mode[1]` to 1 and validate J2 gravity direction.
4. Repeat for `axis_mode[3]` (J4), then `axis_mode[4]` (J5).
5. Change one validated axis at a time from mode 1 to mode 2.

Every transition captures the current position and ramps torque in over
`ramp_time_s`. Never enable several unverified axes simultaneously.

## State values

- `0`: waiting for first valid feedback from all six joints.
- `1`: position hold.
- `2`: mode transition ramp.
- `3`: active gravity/impedance control.
- `4`: degraded; at least one previously seen motor is offline.

`Arm_Control_Debug.fault_mask` and `saturation_mask` use bit 0 through bit 5
for J1 through J6 and bit 6 for the terminal motor where applicable.

## UART7 VOFA channels

UART7 sends JustFloat at 115200 baud and 100 Hz.

| Channel | Signal |
| --- | --- |
| 0-5 | J2 target, position, velocity, gravity torque, impedance torque, command torque |
| 6-11 | J4 target, position, velocity, gravity torque, impedance torque, command torque |
| 12-17 | J5 target, position, velocity, gravity torque, impedance torque, command torque |
| 18 | controller state |
| 19 | fault mask |

Telemetry is best-effort. A busy UART drops the current frame rather than
blocking the 1 kHz motor-control task.
