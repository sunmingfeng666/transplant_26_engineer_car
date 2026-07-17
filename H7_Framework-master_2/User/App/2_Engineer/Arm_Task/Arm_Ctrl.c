#include "Arm_Ctrl.h"

#include <math.h>
#include <string.h>

#include "Classic_Control.h"
#include "Arm_OneClick.h"
#include "DM_Motor.h"
#include "Arm_MatlabDebug.h"
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

/*
 * 上电默认"正常位"(收起位)，J1~J6 弧度，取自老代码 Move_Task/One_Click 正常模式基准。
 * 注：老代码 J4=1.8637 超过本代码上限 ARM_J4_MAX(1.848)，此处直接取合法上限。
 */
static const float ARM_HOME_POSE[ARM_JOINT_COUNT] = {
    -0.8310f, -0.0753f, 0.0811f, 1.848f, -0.8379f, -2.6858f
};

#define ARM_JOINT_SPEED           2.0f
#define ARM_DBUS_STEP             0.000005f
#define ARM_VT13_STEP             0.00005f
#define ARM_DBUS_CLAMP_THRESHOLD  300
#define ARM_VT13_BUTTON_RATE      0.35f
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

static uint8_t Arm_IsDisabled(void)
{
#if ARM_CONTROL_BUILD_MODE == ARM_BUILD_MODE_DISABLED
    return 1U;
#else
    return 0U;
#endif
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

#if ARM_CONTROL_BUILD_MODE == ARM_BUILD_MODE_NORMAL
static uint8_t Arm_IsForcePvAxis(uint8_t axis)
{
    return axis == ARM_AXIS_J1;
}
#endif

static uint8_t Arm_RequestedMode(uint8_t axis)
{
#if ARM_CONTROL_BUILD_MODE == ARM_BUILD_MODE_DISABLED
    (void)axis;
    return ARM_MODE_DISABLED;
#elif ARM_CONTROL_BUILD_MODE == ARM_BUILD_MODE_GRAVITY_ONLY
    /* 当前只有J2/J4/J5具备已验证重力模型；其它关节继续使用电机内部位置环保持。 */
    if (axis == ARM_AXIS_J2 || axis == ARM_AXIS_J4 || axis == ARM_AXIS_J5) {
        return ARM_MODE_GRAVITY;
    }
    return ARM_MODE_POSITION;
#else
    uint8_t mode;

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
#endif
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
        Arm_JointController_ResetCascade(axis);
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
    Arm_JointController_ResetCascade(axis);
    Arm_Control_Debug.ramp[axis] = (requested_mode == ARM_MODE_POSITION) ? 1.0f : 0.0f;
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

#if ARM_CONTROL_BUILD_MODE != ARM_BUILD_MODE_GRAVITY_ONLY
static void Arm_LimitTargets(void)
{
    s_target[ARM_AXIS_J1] = Arm_Clamp(s_target[ARM_AXIS_J1], P_MIN, P_MAX);
    s_target[ARM_AXIS_J2] = Arm_Clamp(s_target[ARM_AXIS_J2], ARM_J2_MIN, ARM_J2_MAX);
    s_target[ARM_AXIS_J3] = Arm_Clamp(s_target[ARM_AXIS_J3], P_MIN, P_MAX);
    s_target[ARM_AXIS_J4] = Arm_Clamp(s_target[ARM_AXIS_J4], ARM_J4_MIN, ARM_J4_MAX);
    s_target[ARM_AXIS_J5] = Arm_Clamp(s_target[ARM_AXIS_J5], ARM_J5_MIN, ARM_J5_MAX);
    s_target[ARM_AXIS_J6] = Arm_Clamp(s_target[ARM_AXIS_J6], P_MIN, P_MAX);
}
#endif

static void Arm_UpdateRemoteTarget(const DBUS_Typedef *dbus,
                                   const VT13_Typedef *vt13,
                                   float dt_s)
{
#if ARM_CONTROL_BUILD_MODE == ARM_BUILD_MODE_GRAVITY_ONLY
    /* 独立重力调试固件不接受遥控目标，防止测试过程中位置关节突然运动。 */
    (void)dbus;
    (void)vt13;
    (void)dt_s;
#else
    float step_scale;

    /* 全局失能期间禁止累计目标，重新使能时再从实际位置捕获。 */
    if (Arm_IsDisabled()) return;
    step_scale = Arm_Clamp(dt_s, 0.0f, 0.01f) / 0.001f;


    if (vt13 != NULL && vt13->offline.is_online) {
        if (vt13->Remote.mode_sw == 2U) {
            s_target[ARM_AXIS_J1] -= (float)vt13->Remote.Channel[0] * ARM_VT13_STEP * step_scale;
            s_target[ARM_AXIS_J2] += (float)vt13->Remote.Channel[1] * ARM_VT13_STEP * step_scale;
            s_target[ARM_AXIS_J3] += (float)vt13->Remote.Channel[3] * ARM_VT13_STEP * step_scale;
            s_target[ARM_AXIS_J4] += (float)vt13->Remote.Channel[2] * ARM_VT13_STEP * step_scale;
            s_target[ARM_AXIS_J5] += ((float)vt13->Remote.fn_1 - (float)vt13->Remote.fn_2) *
                                     ARM_VT13_BUTTON_RATE * dt_s;
            s_target[ARM_AXIS_J6] += (float)vt13->Remote.wheel * ARM_VT13_STEP * step_scale;
            Arm_LimitTargets();
        }

        return;
    }

    if (dbus == NULL || !dbus->offline.is_online) return;

    /* DBUS 挡位解耦：只有 S1=2 才允许机械臂读取摇杆。 */
    if (dbus->Remote.S1 != 2U) return;

    if (dbus->Remote.S2 == 1U) {
        s_target[ARM_AXIS_J1] -= (float)dbus->Remote.CH0 * ARM_DBUS_STEP * step_scale;
        s_target[ARM_AXIS_J2] += (float)dbus->Remote.CH1 * ARM_DBUS_STEP * step_scale;
        s_target[ARM_AXIS_J3] += (float)dbus->Remote.CH2 * ARM_DBUS_STEP * step_scale;
        s_target[ARM_AXIS_J4] += (float)dbus->Remote.CH3 * ARM_DBUS_STEP * step_scale;
    } else if (dbus->Remote.S2 == 2U) {
        s_target[ARM_AXIS_J5] += (float)dbus->Remote.CH0 * ARM_DBUS_STEP * step_scale;
        s_target[ARM_AXIS_J6] += (float)dbus->Remote.CH1 * ARM_DBUS_STEP * step_scale;
        if (dbus->Remote.CH2 > ARM_DBUS_CLAMP_THRESHOLD) {
            s_clamp_close = 1U;
        } else if (dbus->Remote.CH2 < -ARM_DBUS_CLAMP_THRESHOLD) {
            s_clamp_close = 0U;
        }
    }
    Arm_LimitTargets();
#endif
}

uint8_t Engineer_Arm_Init(void)
{
    /* 上电默认目标=正常位(收起位)，而非零位。 */
    memcpy(s_target, ARM_HOME_POSE, sizeof(s_target));
    memset(s_external_pid, 0, sizeof(s_external_pid));
    memset(s_prev_online, 0, sizeof(s_prev_online));
    memset((void *)&Arm_Control_Debug, 0, sizeof(Arm_Control_Debug));
    for (uint8_t axis = 0U; axis < ARM_JOINT_COUNT; axis++) {
        s_active_mode[axis] = ARM_MODE_UNKNOWN;
        Arm_InitExternalPid(axis);
        Arm_JointController_ResetCascade(axis);
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
                       const VT13_Typedef *vt13,
                       float dt_s)
{
    uint16_t online_mask = 0U;
    uint16_t fault_mask = 0U;
    uint16_t saturation_mask = 0U;
    uint8_t any_active = 0U;
    uint8_t any_ramping = 0U;
    uint8_t retry_due;
    uint32_t now;
    float joint_position[ARM_JOINT_COUNT];

    if (feedback == NULL) return;
    dt_s = Arm_Clamp(dt_s, 0.0001f, 0.01f);
    now = HAL_GetTick();
    retry_due = (now - s_last_retry_tick) >= ARM_RETRY_PERIOD_MS;
    Arm_Control_Debug.remote_online =
        ((dbus != NULL && dbus->offline.is_online) ||
         (vt13 != NULL && vt13->offline.is_online)) ? 1U : 0U;

    for (uint8_t axis = 0U; axis < ARM_JOINT_COUNT; axis++) {
        const DM_MOTOR_DATA_Typedef *motor = Arm_GetFeedback(feedback, axis);
        uint8_t requested_mode = Arm_RequestedMode(axis);
        uint8_t online = motor->offline.is_online && isfinite(motor->pos) &&
                         isfinite(motor->vel) && isfinite(motor->tor);

        Arm_Control_Debug.position[axis] = motor->pos;
        Arm_Control_Debug.velocity[axis] = motor->vel;
        joint_position[axis] = motor->pos;

        if (!online) {
            fault_mask |= (uint16_t)(1U << axis);
            s_prev_online[axis] = 0U;
            PID_Clear(&s_external_pid[axis]);
            Arm_JointController_ResetCascade(axis);
            if (retry_due) {
                Arm_ConfigureAxis(axis, requested_mode);
            }
            continue;
        }

        online_mask |= (uint16_t)(1U << axis);
        if (!s_prev_online[axis]) {
#if ARM_CONTROL_BUILD_MODE == ARM_BUILD_MODE_GRAVITY_ONLY
            /* 重力调试时J1/J3/J6保持上电瞬间位置，不主动前往正常收起位。 */
            s_target[axis] = motor->pos;
#else
            /* 上电首次上线：目标设为正常位，机械臂主动收拢到收起姿态(而非停在当前位置)。 */
            s_target[axis] = ARM_HOME_POSE[axis];
#endif
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

    Arm_UpdateRemoteTarget(dbus, vt13, dt_s);

#if ARM_MATLAB_DEBUG_ENABLE && (ARM_CONTROL_BUILD_MODE != ARM_BUILD_MODE_GRAVITY_ONLY)
    if (Arm_MatlabDebug_ApplyTarget(s_target, ARM_JOINT_COUNT)) {
        Arm_LimitTargets();
    }
#endif

#if ARM_CONTROL_BUILD_MODE != ARM_BUILD_MODE_GRAVITY_ONLY
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
#endif

    for (uint8_t axis = 0U; axis < ARM_JOINT_COUNT; axis++) {
        const DM_MOTOR_DATA_Typedef *motor = Arm_GetFeedback(feedback, axis);
        uint8_t mode = s_active_mode[axis];
        float gravity_tau = 0.0f;
        float impedance_tau = 0.0f;
        float command_tau = 0.0f;

        Arm_Control_Debug.target[axis] = s_target[axis];
        Arm_Control_Debug.position_error[axis] =
            ((online_mask & (1U << axis)) != 0U) ? (s_target[axis] - motor->pos) : 0.0f;
        Arm_Control_Debug.cascade_target_velocity[axis] = 0.0f;
        Arm_Control_Debug.cascade_velocity_error[axis] = 0.0f;
        Arm_Control_Debug.cascade_integral_tau[axis] = 0.0f;
        if (mode == ARM_MODE_DISABLED) {
            PID_Clear(&s_external_pid[axis]);
            Arm_JointController_ResetCascade(axis);
            Arm_Control_Debug.gravity_tau[axis] = 0.0f;
            Arm_Control_Debug.impedance_tau[axis] = 0.0f;
            Arm_Control_Debug.command_tau[axis] = 0.0f;
            continue;
        }

        if ((online_mask & (1U << axis)) == 0U || mode == ARM_MODE_UNKNOWN) {
            Arm_JointController_ResetCascade(axis);
            Arm_Control_Debug.gravity_tau[axis] = 0.0f;
            Arm_Control_Debug.impedance_tau[axis] = 0.0f;
            Arm_Control_Debug.command_tau[axis] = 0.0f;
            continue;
        }

        if (mode == ARM_MODE_POSITION) {
            PID_Clear(&s_external_pid[axis]);
            Arm_JointController_ResetCascade(axis);
            Pos_Speed_Ctrl(Arm_GetBus(axis), Arm_GetMotorId(axis), s_target[axis], ARM_JOINT_SPEED);
        } else {
            float ramp_time = Arm_Clamp(Arm_Control_Config.ramp_time_s, 0.1f, 5.0f);
            uint8_t controller_saturated = 0U;
            uint8_t output_saturated = 0U;

            if (ramp_time < 0.1f) ramp_time = 0.1f;
            any_active = 1U;
            Arm_Control_Debug.ramp[axis] += dt_s / ramp_time;
            if (Arm_Control_Debug.ramp[axis] < 1.0f) any_ramping = 1U;
            else Arm_Control_Debug.ramp[axis] = 1.0f;

            switch (mode) {
            case ARM_MODE_GRAVITY:
                gravity_tau = Arm_JointController_Gravity(axis, joint_position);
                PID_Clear(&s_external_pid[axis]);
                break;
            case ARM_MODE_GRAVITY_IMPEDANCE:
                gravity_tau = Arm_JointController_Gravity(axis, joint_position);
                impedance_tau = Arm_CalcExternalPidTorque(axis, s_target[axis], motor->pos);
                break;
            case ARM_MODE_MIT:
                impedance_tau = Arm_CalcExternalPidTorque(axis, s_target[axis], motor->pos);
                break;
            case ARM_MODE_CASCADE: {
                const Arm_Cascade_Output_t cascade =
                    Arm_JointController_Cascade(axis, s_target[axis],
                                                motor->pos, motor->vel, dt_s);
                /* 串级PID输出反馈力矩；已有模型的轴在总力矩中同时叠加重力前馈。 */
                gravity_tau = Arm_JointController_Gravity(axis, joint_position);
                PID_Clear(&s_external_pid[axis]);
                impedance_tau = cascade.torque;
                Arm_Control_Debug.cascade_target_velocity[axis] = cascade.target_velocity;
                Arm_Control_Debug.cascade_velocity_error[axis] = cascade.velocity_error;
                Arm_Control_Debug.cascade_integral_tau[axis] = cascade.integral_tau;
                if (cascade.saturated) controller_saturated = 1U;
                break;
            }
            default:
                PID_Clear(&s_external_pid[axis]);
                Arm_JointController_ResetCascade(axis);
                break;
            }
            command_tau = (gravity_tau + impedance_tau) * Arm_Control_Debug.ramp[axis];
            command_tau = Arm_JointController_LimitTorque(axis, command_tau, &output_saturated);
            MIT_Ctrl(Arm_GetBus(axis), Arm_GetMotorId(axis),
                     s_target[axis], 0.0f, 0.0f, 0.0f, command_tau);
            if (controller_saturated || output_saturated) {
                saturation_mask |= (uint16_t)(1U << axis);
            }
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
        const Arm_OneClick_Output_t *oneclick_output = Arm_OneClick_GetOutput();
        uint8_t clamp_close = s_clamp_close;
        if (oneclick_output != NULL &&
            oneclick_output->active && oneclick_output->clamp_override) {
            clamp_close = oneclick_output->clamp_close;
        }
        Arm_Control_Debug.clamp_close = clamp_close;
        online_mask |= (uint16_t)(1U << 6);
        if (!s_terminal_prev_online) {
            Motor_Mode(&hfdcan3, 0x07U, MIT_MODE, DM_CMD_CLEAR_ERROR);
            Motor_Mode(&hfdcan3, 0x07U, MIT_MODE, DM_CMD_MOTOR_MODE);
            s_terminal_prev_online = 1U;
        }
        MIT_Ctrl(&hfdcan3, 0x07U, 0.0f, 0.0f, 0.0f, 0.0f,
                 clamp_close ? ARM_TERMINAL_CLOSE_TORQUE : ARM_TERMINAL_OPEN_TORQUE);
    } else {
        fault_mask |= (uint16_t)(1U << 6);
        Arm_Control_Debug.clamp_close = 0U;
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
