# Engineer Arm Control

## Control modes

`Arm_Control_Config` is the only runtime tuning entry.

- `master_enable = 0`: all six joints use position-speed mode.
- `master_enable = 1`: J2/J4/J5 follow `axis_mode[n]`; others stay position mode.
- `ARM_CONTROL_BUILD_MODE = ARM_BUILD_MODE_DISABLED`: **disable build** — every
  joint motor and the terminal gripper are sent `DM_CMD_RESET_MODE`. Motors
  release all torque and the arm can be moved by hand. State reports `5`
  (`ARM_STATE_DISABLED`). This compile-time selection overrides
  `master_enable` regardless of its value.
- `ARM_CONTROL_BUILD_MODE = ARM_BUILD_MODE_GRAVITY_ONLY`: **gravity-only
  build** — J2/J4/J5 run their validated gravity models without impedance PID;
  J1/J3/J6 capture and hold their power-on positions with the motor position
  loop. Remote targets, MATLAB targets, and one-click trajectories are ignored.
- `axis_mode[n] = 0`: position-speed mode.
- `axis_mode[n] = 1`: gravity compensation.
- `axis_mode[n] = 2`: gravity compensation plus external impedance.

Only axes 1, 3, and 4 (J2, J4, and J5) accept gravity or impedance modes.
The defaults are safe: `master_enable` is zero and every axis is in position mode.

### Disable build (`ARM_CONTROL_BUILD_MODE = ARM_BUILD_MODE_DISABLED`)

Sets all six joints plus the terminal gripper to `DM_CMD_RESET_MODE`, releasing
all torque. Use this build to hand-pose the arm during a dedicated debug run.
The compile-time selection is declared in `Arm_Ctrl.h`, defaults to
`ARM_BUILD_MODE_NORMAL`, and takes priority over `master_enable`.

- The arm may drop under gravity once torque is released — support it first.
- Disabled motors stop feedback, so they read as offline/fault while disabled;
  this is expected and the state still reports `5` (disabled) with priority.
- To recover, select `ARM_BUILD_MODE_NORMAL`, rebuild, and flash again. The mode
  cannot be changed through the debugger while the firmware is running.

### Gravity-only build (`ARM_CONTROL_BUILD_MODE = ARM_BUILD_MODE_GRAVITY_ONLY`)

This build is intended for isolated gravity-compensation verification. Only
J2, J4, and J5 currently have validated gravity models, so only those axes enter
`ARM_MODE_GRAVITY`. J1, J3, and J6 stay in position mode and capture their
actual positions when feedback first becomes valid. This prevents zero-model
axes from becoming torque-free.

- `master_enable` and `axis_mode[]` do not select the active modes in this build.
- External impedance PID is not calculated for J2/J4/J5.
- Remote, MATLAB, and one-click position targets cannot move the joints.
- Torque limits, offline handling, feedback checks, and torque ramp remain active.
- Support the arm before flashing and first verify the gravity direction with a
  reduced `gravity_scale`.

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
- `5`: disabled; all motors released by the disabled build selection.

`Arm_Control_Debug.fault_mask` and `saturation_mask` use bit 0 through bit 5
for J1 through J6 and bit 6 for the terminal motor where applicable.

## UART7 host port

UART7 is board2's shared host TX port. It always sends JustFloat at 115200
baud and 100 Hz, regardless of the MATLAB debug switch. Use it for VOFA
waveforms or MATLAB co-simulation; edit the single `VOFA_JustFloat` call in
`All_Task.c` to change what the channels carry.

| Channel | Signal |
| --- | --- |
| 0-5 | J1..J6 actual position (rad) — MATLAB reads these six for the 3D model |
| 6-8 | J2 / J4 / J5 target |
| 9-11 | J2 / J4 / J5 velocity |
| 12-14 | J2 / J4 / J5 gravity torque |
| 15-17 | J2 / J4 / J5 command torque |
| 18 | controller state |
| 19 | MATLAB link online flag (0 when debug build is off) |

Telemetry is best-effort. A busy UART drops the current frame rather than
blocking the 1 kHz motor-control task.

## MATLAB debug (test only)

Module `Arm_MatlabDebug.c/.h`, compile switch `ARM_MATLAB_DEBUG_ENABLE`.

- `0`: the whole flow is compiled out. `Engineer_Arm_Task` runs pure DBUS
  logic and UART7 registers no receiver. UART7 TX still works, so VOFA
  waveform viewing is unaffected. Set this for match firmware.
- `1`: UART7 also receives MATLAB downlink frames `single([J1..J6, Inf])`
  and injects J2/J4/J5 targets into the arm controller. Only these three
  axes are used; other channels are reserved.

Runtime enable is `Arm_MatlabDebug_Enable` (default 0). The link auto-falls
back to DBUS if no fresh frame arrives within `ARM_MATLAB_DEBUG_TIMEOUT_MS`
(200 ms). Injected targets still pass through `Arm_LimitTargets`, so joint
limits hold even under a bad downlink.

Recommended bring-up:

1. Build with `ARM_MATLAB_DEBUG_ENABLE = 1`, keep all axes in position mode.
2. Mechanically support the arm, confirm all six joints are online.
3. Run `MATLAB/engineer_arm.m`, set the COM port, confirm ch19 shows online.
4. Set `Arm_MatlabDebug_Enable = 1`.
5. Nudge the J2 slider a small amount; verify direction and sign match.
6. Repeat for J4, then J5. Never move several unverified axes at once.

MATLAB host: `MATLAB/engineer_arm.m`. Its DH parameters are placeholders —
fill in the real engineer-arm values before trusting the 3D pose.
