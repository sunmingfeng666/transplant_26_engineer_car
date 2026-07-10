#include "Arm_Ctrl.h"

#include <math.h>
#include <string.h>

#include "Classic_Control.h"
#include "DM_Motor.h"
#include "fdcan.h"

#define ARM_AXIS_J1 0U
#define ARM_AXIS_J2 1U
#define ARM_AXIS_J3 2U
#define ARM_AXIS_J4 3U
#define ARM_AXIS_J5 4U
#define ARM_AXIS_J6 5U

#define ARM_J2_MIN (-0.532349f)
#define ARM_J2_MAX ( 3.67875862f)
#define ARM_J4_MIN (-1.8297427f)
#define ARM_J4_MAX ( 1.848f)
#define ARM_J5_MIN (-1.76146317f)
#define ARM_J5_MAX ( 1.67410f)

#define ARM_JOINT_SPEED           2.0f
#define ARM_DBUS_STEP             0.00005f
#define ARM_RETRY_PERIOD_MS       100U
#define ARM_TERMINAL_CLOSE_TORQUE 1.0f
#define ARM_TERMINAL_OPEN_TORQUE (-1.2f)
#define ARM_MODE_UNKNOWN          0xFFU

static float s_target[ARM_JOINT_COUNT];
static PID_t s_external_pid[ARM_JOINT_COUNT];
static uint8_t s_active_mode[ARM_JOINT_COUNT];
static uint8_t s_prev_online[ARM_JOINT_COUNT];
static uint8_t s_target_valid_mask;
static uint8_t s_terminal_prev_online;
static uint8_t s_clamp_close;
static uint32_t s_last_retry_tick;

volatile uint8_t Arm_Disable_Enable = 0U;

static uint8_t Arm_IsDisabled(void)
{
    return Arm_Disable_Enable != 0U;
}

static float Arm_Clamp(float value, float min_value, float max_value)
{
    if (!isfinite(value)) return 0.0f;
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static const DM_MOTOR_DATA_Typedef *Arm_GetFeedback(const Arm_Motor_Group_t *feedback, uint8_t axis)
{
    switch (axis) {
    case ARM_AXIS_J1: return &feedback->J1_8009;
    case ARM_AXIS_J2: return &feedback->J2_8009;
    case ARM_AXIS_J3: return &feedback->J3_4340;
    case ARM_AXIS_J4: return &feedback->J4_4340;
    case ARM_AXIS_J5: return &feedback->J5_4310;
    case ARM_AXIS_J6: return &feedback->J6_4310;
    default: return NULL;
    }
}

static FDCAN_HandleTypeDef *Arm_GetBus(uint8_t axis)
{
    return axis <= ARM_AXIS_J3 ? &hfdcan2 : &hfdcan3;
}

static uint16_t Arm_GetMotorId(uint8_t axis)
{
    return (uint16_t)(axis + 1U);
}

static uint8_t Arm_IsForcePvAxis(uint8_t axis)
{
    return axis == ARM_AXIS_J1 || axis == ARM_AXIS_J5;
}

static uint8_t Arm_RequestedMode(uint8_t axis)
{
    uint8_t mode;

    if (Arm_IsDisabled()) {
        return ARM_MODE_DISABLED;
    }
    if (axis >= ARM_JOINT_COUNT) {
        return ARM_MODE_POSITION;
    }
    if (Arm_IsForcePvAxis(axis)) {
        return ARM_MODE_POSITION;
    }
    if (Arm_Control_Config.master_enable != 1U) {
        return ARM_MODE_POSITION;
    }

    mode = Arm_Control_Config.axis_mode[axis];
    if (mode > ARM_MODE_DISABLED) return ARM_MODE_POSITION;
    return mode;
}

static uint16_t Arm_DmMode(uint8_t mode)
{
    if (mode == ARM_MODE_DISABLED) return 0xFFFFU;
    return mode == ARM_MODE_POSITION ? POS_MODE : MIT_MODE;
}

static void Arm_ConfigureAxis(uint8_t axis, uint8_t requested_mode)
{
    FDCAN_HandleTypeDef *bus = Arm_GetBus(axis);
    uint16_t id = Arm_GetMotorId(axis);
    uint16_t new_dm_mode = Arm_DmMode(requested_mode);

    if (requested_mode == ARM_MODE_DISABLED) {
        Motor_Mode(bus, id, MIT_MODE, DM_CMD_RESET_MODE);
        s_active_mode[axis] = ARM_MODE_DISABLED;
        PID_Clear(&s_external_pid[axis]);
        Arm_Control_Debug.ramp[axis] = 0.0f;
        return;
    }

    if (s_active_mode[axis] != ARM_MODE_UNKNOWN &&
        s_active_mode[axis] != ARM_MODE_DISABLED &&
        Arm_DmMode(s_active_mode[axis]) != new_dm_mode) {
        Motor_Mode(bus, id, Arm_DmMode(s_active_mode[axis]), DM_CMD_RESET_MODE);
    }
    Motor_Mode(bus, id, new_dm_mode, DM_CMD_CLEAR_ERROR);
    Motor_Mode(bus, id, new_dm_mode, DM_CMD_MOTOR_MODE);
    s_active_mode[axis] = requested_mode;
    PID_Clear(&s_external_pid[axis]);
    Arm_Control_Debug.ramp[axis] = requested_mode == ARM_MODE_POSITION ? 1.0f : 0.0f;
}

static void Arm_LoadExternalPidParam(uint8_t axis, float kpid[3])
{
    kpid[0] = Arm_Clamp(Arm_Control_Config.impedance_kp[axis], 0.0f, KP_MAX);
    kpid[1] = Arm_Clamp(Arm_Control_Config.impedance_ki[axis], 0.0f, 50.0f);
    kpid[2] = Arm_Clamp(Arm_Control_Config.impedance_kd[axis], 0.0f, KD_MAX);
}

static void Arm_InitExternalPid(uint8_t axis)
{
    float kpid[3];

    if (axis >= ARM_JOINT_COUNT) return;
    Arm_LoadExternalPidParam(axis, kpid);
    PID_Init(&s_external_pid[axis],
             Arm_Clamp(Arm_Control_Config.torque_limit[axis], 0.0f, T_MAX),
             Arm_Clamp(Arm_Control_Config.impedance_i_limit[axis], 0.0f, T_MAX),
             kpid,
             0.0f, 0.0f,
             0.0f, 0.0f,
             0U,
             Integral_Limit);
}

static void Arm_SyncExternalPid(uint8_t axis)
{
    float kpid[3];

    if (axis >= ARM_JOINT_COUNT) return;
    Arm_LoadExternalPidParam(axis, kpid);
    PID_set(&s_external_pid[axis], kpid);
    s_external_pid[axis].MaxOut = Arm_Clamp(Arm_Control_Config.torque_limit[axis], 0.0f, T_MAX);
    s_external_pid[axis].IntegralLimit = Arm_Clamp(Arm_Control_Config.impedance_i_limit[axis], 0.0f, T_MAX);
}

static float Arm_CalcExternalPidTorque(uint8_t axis, float target, float position)
{
    if (axis >= ARM_JOINT_COUNT) {
        return 0.0f;
    }

    Arm_SyncExternalPid(axis);
    return PID_Calculate(&s_external_pid[axis], position, target);
}

static void Arm_LimitTargets(void)
{
    s_target[ARM_AXIS_J1] = Arm_Clamp(s_target[ARM_AXIS_J1], P_MIN, P_MAX);
    s_target[ARM_AXIS_J2] = Arm_Clamp(s_target[ARM_AXIS_J2], ARM_J2_MIN, ARM_J2_MAX);
    s_target[ARM_AXIS_J3] = Arm_Clamp(s_target[ARM_AXIS_J3], P_MIN, P_MAX);
    s_target[ARM_AXIS_J4] = Arm_Clamp(s_target[ARM_AXIS_J4], ARM_J4_MIN, ARM_J4_MAX);
    s_target[ARM_AXIS_J5] = Arm_Clamp(s_target[ARM_AXIS_J5], ARM_J5_MIN, ARM_J5_MAX);
    s_target[ARM_AXIS_J6] = Arm_Clamp(s_target[ARM_AXIS_J6], P_MIN, P_MAX);
}

static void Arm_UpdateDbusTarget(const DBUS_Typedef *dbus, float dt_s)
{
    float step_scale;

    if (dbus == NULL || !dbus->offline.is_online) return;
    step_scale = Arm_Clamp(dt_s, 0.0f, 0.01f) / 0.001f;

    if (dbus->Remote.S1 == 1U) {
        s_target[ARM_AXIS_J1] -= (float)dbus->Remote.CH0 * ARM_DBUS_STEP * step_scale;
        s_target[ARM_AXIS_J2] += (float)dbus->Remote.CH1 * ARM_DBUS_STEP * step_scale;
        s_target[ARM_AXIS_J3] += (float)dbus->Remote.CH2 * ARM_DBUS_STEP * step_scale;
        s_target[ARM_AXIS_J4] += (float)dbus->Remote.CH3 * ARM_DBUS_STEP * step_scale;
    } else if (dbus->Remote.S1 == 2U) {
        s_target[ARM_AXIS_J5] += (float)dbus->Remote.CH0 * ARM_DBUS_STEP * step_scale;
        s_target[ARM_AXIS_J6] += (float)dbus->Remote.CH1 * ARM_DBUS_STEP * step_scale;
    }

    if (dbus->Remote.S2 == 1U) s_clamp_close = 1U;
    else if (dbus->Remote.S2 == 2U) s_clamp_close = 0U;
    Arm_LimitTargets();
}

uint8_t Engineer_Arm_Init(void)
{
    memset(s_target, 0, sizeof(s_target));
    memset(s_external_pid, 0, sizeof(s_external_pid));
    memset(s_prev_online, 0, sizeof(s_prev_online));
    memset((void *)&Arm_Control_Debug, 0, sizeof(Arm_Control_Debug));
    for (uint8_t axis = 0U; axis < ARM_JOINT_COUNT; axis++) {
        s_active_mode[axis] = ARM_MODE_UNKNOWN;
        Arm_InitExternalPid(axis);
    }
    s_target_valid_mask = 0U;
    s_terminal_prev_online = 0U;
    s_clamp_close = 1U;
    s_last_retry_tick = 0U;
    Arm_Control_Debug.state = ARM_STATE_WAIT_FEEDBACK;
    return 0U;
}

void Engineer_Arm_Task(const Arm_Motor_Group_t *feedback,
                       const DBUS_Typedef *dbus,
                       float dt_s)
{
    uint16_t online_mask = 0U;
    uint16_t fault_mask = 0U;
    uint16_t saturation_mask = 0U;
    uint8_t any_active = 0U;
    uint8_t any_ramping = 0U;
    uint8_t retry_due;
    uint32_t now;

    if (feedback == NULL) return;
    dt_s = Arm_Clamp(dt_s, 0.0001f, 0.01f);
    now = HAL_GetTick();
    retry_due = (now - s_last_retry_tick) >= ARM_RETRY_PERIOD_MS;
    Arm_Control_Debug.remote_online = (dbus != NULL && dbus->offline.is_online) ? 1U : 0U;

    for (uint8_t axis = 0U; axis < ARM_JOINT_COUNT; axis++) {
        const DM_MOTOR_DATA_Typedef *motor = Arm_GetFeedback(feedback, axis);
        uint8_t requested_mode = Arm_RequestedMode(axis);
        uint8_t online = motor->offline.is_online && isfinite(motor->pos) &&
                         isfinite(motor->vel) && isfinite(motor->tor);

        Arm_Control_Debug.position[axis] = motor->pos;
        Arm_Control_Debug.velocity[axis] = motor->vel;

        if (!online) {
            fault_mask |= (uint16_t)(1U << axis);
            s_prev_online[axis] = 0U;
            PID_Clear(&s_external_pid[axis]);
            if (retry_due) {
                Arm_ConfigureAxis(axis, requested_mode);
            }
            continue;
        }

        online_mask |= (uint16_t)(1U << axis);
        if (!s_prev_online[axis]) {
            s_target[axis] = motor->pos;
            s_target_valid_mask |= (uint8_t)(1U << axis);
            Arm_ConfigureAxis(axis, requested_mode);
            s_prev_online[axis] = 1U;
        } else if (s_active_mode[axis] != requested_mode) {
            s_target[axis] = motor->pos;
            Arm_ConfigureAxis(axis, requested_mode);
        }
    }

    if (retry_due) {
        s_last_retry_tick = now;
    }

    Arm_UpdateDbusTarget(dbus, dt_s);

    for (uint8_t axis = 0U; axis < ARM_JOINT_COUNT; axis++) {
        const DM_MOTOR_DATA_Typedef *motor = Arm_GetFeedback(feedback, axis);
        uint8_t mode = s_active_mode[axis];
        float gravity_tau = 0.0f;
        float impedance_tau = 0.0f;
        float command_tau = 0.0f;

        Arm_Control_Debug.target[axis] = s_target[axis];
        if (mode == ARM_MODE_DISABLED) {
            PID_Clear(&s_external_pid[axis]);
            Arm_Control_Debug.gravity_tau[axis] = 0.0f;
            Arm_Control_Debug.impedance_tau[axis] = 0.0f;
            Arm_Control_Debug.command_tau[axis] = 0.0f;
            continue;
        }

        if ((online_mask & (1U << axis)) == 0U || mode == ARM_MODE_UNKNOWN) {
            Arm_Control_Debug.gravity_tau[axis] = 0.0f;
            Arm_Control_Debug.impedance_tau[axis] = 0.0f;
            Arm_Control_Debug.command_tau[axis] = 0.0f;
            continue;
        }

        if (mode == ARM_MODE_POSITION) {
            PID_Clear(&s_external_pid[axis]);
            Pos_Speed_Ctrl(Arm_GetBus(axis), Arm_GetMotorId(axis), s_target[axis], ARM_JOINT_SPEED);
        } else {
            float ramp_time = Arm_Clamp(Arm_Control_Config.ramp_time_s, 0.1f, 5.0f);
            uint8_t saturated = 0U;

            if (ramp_time < 0.1f) ramp_time = 0.1f;
            any_active = 1U;
            Arm_Control_Debug.ramp[axis] += dt_s / ramp_time;
            if (Arm_Control_Debug.ramp[axis] < 1.0f) any_ramping = 1U;
            else Arm_Control_Debug.ramp[axis] = 1.0f;

            switch (mode) {
            case ARM_MODE_GRAVITY:
                gravity_tau = Arm_JointController_Gravity(
                    axis, feedback->J2_8009.pos, feedback->J4_4340.pos, feedback->J5_4310.pos);
                PID_Clear(&s_external_pid[axis]);
                break;
            case ARM_MODE_GRAVITY_IMPEDANCE:
                gravity_tau = Arm_JointController_Gravity(
                    axis, feedback->J2_8009.pos, feedback->J4_4340.pos, feedback->J5_4310.pos);
                impedance_tau = Arm_CalcExternalPidTorque(axis, s_target[axis], motor->pos);
                break;
            case ARM_MODE_MIT:
                impedance_tau = Arm_CalcExternalPidTorque(axis, s_target[axis], motor->pos);
                break;
            default:
                PID_Clear(&s_external_pid[axis]);
                break;
            }
            command_tau = (gravity_tau + impedance_tau) * Arm_Control_Debug.ramp[axis];
            command_tau = Arm_JointController_LimitTorque(axis, command_tau, &saturated);
            if (saturated) saturation_mask |= (uint16_t)(1U << axis);

            MIT_Ctrl(Arm_GetBus(axis), Arm_GetMotorId(axis),
                     s_target[axis], 0.0f, 0.0f, 0.0f, command_tau);
        }

        Arm_Control_Debug.gravity_tau[axis] = gravity_tau;
        Arm_Control_Debug.impedance_tau[axis] = impedance_tau;
        Arm_Control_Debug.command_tau[axis] = command_tau;
    }

    if (Arm_IsDisabled()) {
        if (s_terminal_prev_online) {
            Motor_Mode(&hfdcan3, 0x07U, MIT_MODE, DM_CMD_RESET_MODE);
            s_terminal_prev_online = 0U;
        }
    } else if (feedback->Terminal_3507.offline.is_online) {
        online_mask |= (uint16_t)(1U << 6);
        if (!s_terminal_prev_online) {
            Motor_Mode(&hfdcan3, 0x07U, MIT_MODE, DM_CMD_CLEAR_ERROR);
            Motor_Mode(&hfdcan3, 0x07U, MIT_MODE, DM_CMD_MOTOR_MODE);
            s_terminal_prev_online = 1U;
        }
        MIT_Ctrl(&hfdcan3, 0x07U, 0.0f, 0.0f, 0.0f, 0.0f,
                 s_clamp_close ? ARM_TERMINAL_CLOSE_TORQUE : ARM_TERMINAL_OPEN_TORQUE);
    } else {
        fault_mask |= (uint16_t)(1U << 6);
        s_terminal_prev_online = 0U;
        if (retry_due) {
            Motor_Mode(&hfdcan3, 0x07U, MIT_MODE, DM_CMD_CLEAR_ERROR);
            Motor_Mode(&hfdcan3, 0x07U, MIT_MODE, DM_CMD_MOTOR_MODE);
        }
    }

    Arm_Control_Debug.online_mask = online_mask;
    Arm_Control_Debug.fault_mask = fault_mask;
    Arm_Control_Debug.saturation_mask = saturation_mask;

    if (Arm_IsDisabled()) Arm_Control_Debug.state = ARM_STATE_DISABLED;
    else if (s_target_valid_mask != 0x3FU) Arm_Control_Debug.state = ARM_STATE_WAIT_FEEDBACK;
    else if (fault_mask != 0U) Arm_Control_Debug.state = ARM_STATE_DEGRADED;
    else if (any_ramping) Arm_Control_Debug.state = ARM_STATE_MODE_RAMP;
    else if (any_active) Arm_Control_Debug.state = ARM_STATE_ACTIVE;
    else Arm_Control_Debug.state = ARM_STATE_POSITION_HOLD;
}
