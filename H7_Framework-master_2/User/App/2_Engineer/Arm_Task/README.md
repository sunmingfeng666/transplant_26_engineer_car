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
- `ARM_CONTROL_BUILD_MODE = ARM_BUILD_MODE_CALIBRATION`: **zero calibration
  build** — all six joints capture and hold the first valid feedback instead of
  moving to the folded HOME. DBUS small-step control remains available; MATLAB
  injection and all one-click actions are compiled out.
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

### Zero calibration build (`ARM_CONTROL_BUILD_MODE = ARM_BUILD_MODE_CALIBRATION`)

Use this build before exporting any real-arm pre-grasp trajectory. Mechanically
support the arm at the confirmed physical zero, build with the calibration mode,
and verify one joint at a time with a `+0.05 rad` logical command. Record
`motor_direction` as `+1` for the same direction or `-1` for the opposite
direction, then derive `motor_ratio` from motor-feedback change / real joint
change. Do not send the DM `ZERO_POSITION` command.

The export gate remains closed until all six directions are exactly `+1/-1`,
all six ratios are positive, and `reach_pregrasp/home_config.json` sets
`calibration.valid` to true. Re-run MATLAB `reach_planner`, then run
`reach_pregrasp/export_pregrasp_firmware.py`; this replaces the guarded
`Arm_PregraspTrajectory.inc` placeholder with checked coefficients.

### Pre-grasp one-click interface (Ozone)

Set `Arm_Control_Config.oneclick_pregrasp_unit` to `1..6`, then write one request
to `Arm_Control_Config.oneclick_request`:

| Request | Motion |
| --- | --- |
| `8` | folded HOME -> physical/logical zero |
| `9` | zero -> selected pre-grasp and hold |
| `10` | selected pre-grasp -> zero |
| `11` | zero -> folded HOME |

Set `Arm_Control_Config.oneclick_abort=1` to stop. Observe
`Arm_Control_Debug.oneclick_result`: `7` is `BAD_START`, `8` is
`TRACKING_FAULT`, and `9` is `CALIBRATION_REQUIRED`. Requests are accepted only
with all six axes online. Requests 8..11 additionally require the actual start
to match the directed trajectory start within `0.08 rad`; tracking error above
`0.20 rad` for `0.20 s` stops the action and holds the measured position.

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

| Channel | CSV column | Signal |
| --- | --- | --- |
| 0/1/2 | CH0/CH1/CH2 | J1 target / position / error (rad) |
| 3/4/5 | CH3/CH4/CH5 | J2 target / position / error (rad) |
| 6/7/8 | CH6/CH7/CH8 | J3 target / position / error (rad) |
| 9/10/11 | CH9/CH10/CH11 | J4 target / position / error (rad) |
| 12/13/14 | CH12/CH13/CH14 | J5 target / position / error (rad) |
| 15/16/17 | CH15/CH16/CH17 | J6 target / position / error (rad) |
| 18 | CH18 | Six-axis online bitmap; normal value is 63 (`0x3F`) |
| 19 | CH19 | Six-axis fault bitmap; normal value is 0 |

For calibration CSV capture, keep all 20 channels enabled. Start recording before
moving a joint and keep at least two seconds of stationary data before and after
each movement. Do not rename or reorder the columns before sharing the CSV.

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
