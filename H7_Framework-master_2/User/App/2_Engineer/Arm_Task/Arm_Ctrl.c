#include "Arm_Ctrl.h"

#include <math.h>
#include <string.h>

#include "Arm_MatlabDebug.h"
#include "Arm_OneClick.h"
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
static uint8_t s_active_mode[ARM_JOINT_COUNT];
static uint8_t s_prev_online[ARM_JOINT_COUNT];
static uint8_t s_target_valid_mask;
static uint8_t s_terminal_prev_online;
static uint8_t s_clamp_close;
static uint32_t s_last_retry_tick;

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

static uint8_t Arm_IsCompensatedAxis(uint8_t axis)
{
    return axis == ARM_AXIS_J2 || axis == ARM_AXIS_J4 || axis == ARM_AXIS_J5;
}

static uint8_t Arm_RequestedMode(uint8_t axis)
{
    uint8_t mode;

    /* 如果 master_enable == 0xFF，所有轴进入失能模式 */
    if (Arm_Control_Config.master_enable == 0xFFU) {
        return ARM_MODE_DISABLED;
    }

    if (!Arm_IsCompensatedAxis(axis) || Arm_Control_Config.master_enable == 0U) {
        return ARM_MODE_POSITION;
    }

    mode = Arm_Control_Config.axis_mode[axis];
    if (mode > ARM_MODE_GRAVITY_IMPEDANCE) return ARM_MODE_POSITION;
    return mode;
}

static uint16_t Arm_DmMode(uint8_t mode)
{
    if (mode == ARM_MODE_DISABLED) return 0xFFFFU;  /* 失能模式特殊标记 */
    return mode == ARM_MODE_POSITION ? POS_MODE : MIT_MODE;
}

static void Arm_ConfigureAxis(uint8_t axis, uint8_t requested_mode)
{
    FDCAN_HandleTypeDef *bus = Arm_GetBus(axis);
    uint16_t id = Arm_GetMotorId(axis);
    uint16_t new_dm_mode = Arm_DmMode(requested_mode);

    /* 如果请求失能模式，直接发送失能命令 */
    if (requested_mode == ARM_MODE_DISABLED) {
        Motor_Mode(bus, id, MIT_MODE, DM_CMD_RESET_MODE);
        s_active_mode[axis] = ARM_MODE_DISABLED;
        Arm_Control_Debug.ramp[axis] = 0.0f;
        return;
    }

    /* 切换 DM 硬件模式前先复位旧模式；但失能模式(ARM_MODE_DISABLED)在硬件上
     * 已等同复位状态，其 Arm_DmMode 为 0xFFFF 无效标记，不能用来构造 CAN ID，故跳过。 */
    if (s_active_mode[axis] != ARM_MODE_UNKNOWN &&
        s_active_mode[axis] != ARM_MODE_DISABLED &&
        Arm_DmMode(s_active_mode[axis]) != new_dm_mode) {
        Motor_Mode(bus, id, Arm_DmMode(s_active_mode[axis]), DM_CMD_RESET_MODE);
    }
    Motor_Mode(bus, id, new_dm_mode, DM_CMD_CLEAR_ERROR);
    Motor_Mode(bus, id, new_dm_mode, DM_CMD_MOTOR_MODE);
    s_active_mode[axis] = requested_mode;
    Arm_Control_Debug.ramp[axis] = requested_mode == ARM_MODE_POSITION ? 1.0f : 0.0f;
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
    memset(s_prev_online, 0, sizeof(s_prev_online));
    memset((void *)&Arm_Control_Debug, 0, sizeof(Arm_Control_Debug));
    for (uint8_t axis = 0U; axis < ARM_JOINT_COUNT; axis++) {
        s_active_mode[axis] = ARM_MODE_UNKNOWN;
    }
    s_target_valid_mask = 0U;
    s_terminal_prev_online = 0U;
    s_clamp_close = 1U;
    s_last_retry_tick = 0U;
    Arm_Control_Debug.state = ARM_STATE_WAIT_FEEDBACK;
    Arm_OneClick_Init();
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

#if ARM_MATLAB_DEBUG_ENABLE
    /*
     * MATLAB 联调（仅测试用）：使能且链路在线时覆盖 J2/J4/J5 目标，
     * 覆盖后再钳一次限位保证安全；未使能/掉线则上面 DBUS 逻辑照常生效。
     * 比赛固件编译开关关掉，本段代码从固件中消失。
     */
    if (Arm_MatlabDebug_ApplyTarget(s_target, ARM_JOINT_COUNT)) {
        Arm_LimitTargets();
    }
#endif

    /*
     * 一键动作引擎：激活时沿轨迹覆盖 s_target[]（优先级高于 DBUS/MATLAB），
     * 覆盖后再钳一次限位。空闲则不动 s_target[]，上面逻辑照常生效。
     * 仅在关节目标已全部有效、且未处于全失能模式时允许执行。
     */
    {
        float current_pos[ARM_JOINT_COUNT];
        for (uint8_t axis = 0U; axis < ARM_JOINT_COUNT; axis++) {
            current_pos[axis] = Arm_GetFeedback(feedback, axis)->pos;
        }
        if (s_target_valid_mask == 0x3FU && Arm_Control_Config.master_enable != 0xFFU) {
            if (Arm_OneClick_Update(current_pos, s_target, ARM_JOINT_COUNT)) {
                Arm_LimitTargets();
            }
        } else if (Arm_Control_Debug.oneclick_active) {
            /* 条件不再满足（掉线/失能）时安全中止。 */
            Arm_Control_Config.oneclick_abort = 1U;
            Arm_OneClick_Update(current_pos, s_target, ARM_JOINT_COUNT);
        }
    }

    for (uint8_t axis = 0U; axis < ARM_JOINT_COUNT; axis++) {
        const DM_MOTOR_DATA_Typedef *motor = Arm_GetFeedback(feedback, axis);
        uint8_t mode = s_active_mode[axis];
        float gravity_tau = 0.0f;
        float impedance_tau = 0.0f;
        float command_tau = 0.0f;

        Arm_Control_Debug.target[axis] = s_target[axis];

        /* 失能模式下不发送任何控制命令 */
        if (mode == ARM_MODE_DISABLED) {
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
            Pos_Speed_Ctrl(Arm_GetBus(axis), Arm_GetMotorId(axis), s_target[axis], ARM_JOINT_SPEED);
        } else {
            float ramp_time = Arm_Clamp(Arm_Control_Config.ramp_time_s, 0.1f, 5.0f);
            uint8_t saturated = 0U;

            if (ramp_time < 0.1f) ramp_time = 0.1f;
            any_active = 1U;
            Arm_Control_Debug.ramp[axis] += dt_s / ramp_time;
            if (Arm_Control_Debug.ramp[axis] < 1.0f) any_ramping = 1U;
            else Arm_Control_Debug.ramp[axis] = 1.0f;

            gravity_tau = Arm_JointController_Gravity(
                axis, feedback->J2_8009.pos, feedback->J4_4340.pos, feedback->J5_4310.pos);
            if (mode == ARM_MODE_GRAVITY_IMPEDANCE) {
                impedance_tau = Arm_JointController_Impedance(
                    axis, s_target[axis], motor->pos, motor->vel);
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

    /* 如果处于失能模式，末端夹爪也失能 */
    if (Arm_Control_Config.master_enable == 0xFFU) {
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

    if (Arm_Control_Config.master_enable == 0xFFU) Arm_Control_Debug.state = ARM_STATE_DISABLED;
    else if (s_target_valid_mask != 0x3FU) Arm_Control_Debug.state = ARM_STATE_WAIT_FEEDBACK;
    else if (fault_mask != 0U) Arm_Control_Debug.state = ARM_STATE_DEGRADED;
    else if (any_ramping) Arm_Control_Debug.state = ARM_STATE_MODE_RAMP;
    else if (any_active) Arm_Control_Debug.state = ARM_STATE_ACTIVE;
    else Arm_Control_Debug.state = ARM_STATE_POSITION_HOLD;
}
