#include "Arm_JointController.h"

#include <math.h>

#include "DM_Motor.h"

#define ARM_AXIS_J2 1U
#define ARM_AXIS_J3 2U
#define ARM_AXIS_J4 3U
#define ARM_AXIS_J5 4U
#define ARM_AXIS_J6 5U

#define ARM_CASCADE_POSITION_KP_MAX 100.0f
#define ARM_CASCADE_VELOCITY_KP_MAX 10.0f
#define ARM_CASCADE_VELOCITY_KI_MAX 100.0f

static float s_cascade_integral_tau[ARM_JOINT_COUNT];

volatile Arm_Control_Config_t Arm_Control_Config = {
    .master_enable = 1U,
    /* J1强制PV；J2~J6运行位置-速度串级PID，其中J2/J4/J5叠加重力前馈，串级轴均由MIT模式下发力矩。 */
    .axis_mode = {
        ARM_MODE_POSITION, ARM_MODE_CASCADE, ARM_MODE_CASCADE,
        ARM_MODE_CASCADE, ARM_MODE_CASCADE, ARM_MODE_CASCADE,
    },
    .gravity_scale = {0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f},
    .impedance_kp = {0.0f, 16.0f, 12.0f, 12.0f, 12.0f, 8.0f},
    .impedance_ki = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .impedance_kd = {0.0f, 4.0f, 4.0f, 1.0f, 3.0f, 3.0f},
    .impedance_i_limit = {0.0f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f},
    .torque_limit = {20.0f, 20.0f, 20.0f, 20.0f, 20.0f, 2.0f},
    /* 新增轴先关闭积分并使用保守增益；J6保留已经调过的参数。 */
    .cascade_position_kp = {
        [ARM_AXIS_J2] = 15.0f, [ARM_AXIS_J3] = 8.0f,
        [ARM_AXIS_J4] = 16.0f, [ARM_AXIS_J5] = 22.0f,
        [ARM_AXIS_J6] = 10.0f,
    },
    .cascade_velocity_kp = {
        [ARM_AXIS_J2] = 2.3f, [ARM_AXIS_J3] = 2.1f,
        [ARM_AXIS_J4] = 2.0f, [ARM_AXIS_J5] = 1.0f,
        [ARM_AXIS_J6] = 1.0f,
    },
    .cascade_velocity_ki = {[ARM_AXIS_J2] = 0.01f,[ARM_AXIS_J4] = 0.01f
        ,[ARM_AXIS_J5] = 0.03f,[ARM_AXIS_J6] = 2.0f},
    .cascade_velocity_limit = {
        [ARM_AXIS_J2] = 1.8f, [ARM_AXIS_J3] = 10.0f,
        [ARM_AXIS_J4] = 6.0f, [ARM_AXIS_J5] = 8.0f,
        [ARM_AXIS_J6] = 10.0f,
    },
    .cascade_integral_limit = {
        [ARM_AXIS_J2] = 1.0f, [ARM_AXIS_J3] = 0.8f,
        [ARM_AXIS_J4] = 0.6f, [ARM_AXIS_J5] = 0.5f,
        [ARM_AXIS_J6] = 0.5f,
    },
    .ramp_time_s = 1.0f,
    .gravity = {
        [ARM_AXIS_J2] = {
            .a = 1.7062f, .b = -0.0653f, .c = -0.0309f,
            .d = 0.4963f, .e = -0.1270f,
            .min_rad = 0.48f, .max_rad = 3.25f,
        },
        [ARM_AXIS_J4] = {
            .a = -1.3902f, .b = 0.1950f, .c = 0.9557f,
            .d = -0.0593f, .e = -1.6175f,
            .f = 1.1773f, .g = -0.2333f,
            .min_rad = -1.80f, .max_rad = 1.74f,
        },
        [ARM_AXIS_J5] = {
            .a = 0.6250f, .b = 0.8513f, .c = -0.0244f,
            .d = 0.0375f, .e = -0.5463f,
            .min_rad = -1.59f, .max_rad = 1.60f,
        },
    },
    // 仅 RESET 动作用此默认槽位；存/取矿(composite)会按占用位图自动选槽并覆盖此值。
    .oneclick_store_slot = 1U,
};

volatile Arm_Control_Debug_t Arm_Control_Debug = {
    .state = ARM_STATE_WAIT_FEEDBACK,
};

float Arm_JointController_Gravity(uint8_t axis, const float joint_position[ARM_JOINT_COUNT])
{
    return Arm_JointAlgo_Gravity(axis,
                                 ARM_JOINT_COUNT,
                                 Arm_Control_Config.gravity,
                                 Arm_Control_Config.gravity_scale,
                                 joint_position);
}

float Arm_JointController_Impedance(uint8_t axis, float target, float position, float velocity)
{
    if (axis >= ARM_JOINT_COUNT) {
        return 0.0f;
    }

    return Arm_JointAlgo_Impedance(target,
                                   position,
                                   velocity,
                                   Arm_Control_Config.impedance_kp[axis],
                                   Arm_Control_Config.impedance_kd[axis],
                                   KP_MAX,
                                   KD_MAX);
}

void Arm_JointController_ResetCascade(uint8_t axis)
{
    if (axis >= ARM_JOINT_COUNT) return;
    s_cascade_integral_tau[axis] = 0.0f;
}

Arm_Cascade_Output_t Arm_JointController_Cascade(uint8_t axis,
                                                 float target,
                                                 float position,
                                                 float velocity,
                                                 float dt_s)
{
    Arm_Cascade_Output_t output = {0};
    float position_kp;
    float velocity_kp;
    float velocity_ki;
    float velocity_limit;
    float integral_limit;
    float torque_limit;
    float candidate_integral;
    float proportional_tau;
    float candidate_tau;

    if (axis >= ARM_JOINT_COUNT ||
        !isfinite(target) || !isfinite(position) ||
        !isfinite(velocity) || !isfinite(dt_s)) {
        Arm_JointController_ResetCascade(axis);
        return output;
    }

    dt_s = Arm_JointAlgo_ClampFinite(dt_s, 0.0001f, 0.01f, 0.001f);
    position_kp = Arm_JointAlgo_ClampFinite(
        Arm_Control_Config.cascade_position_kp[axis],
        0.0f, ARM_CASCADE_POSITION_KP_MAX, 0.0f);
    velocity_kp = Arm_JointAlgo_ClampFinite(
        Arm_Control_Config.cascade_velocity_kp[axis],
        0.0f, ARM_CASCADE_VELOCITY_KP_MAX, 0.0f);
    velocity_ki = Arm_JointAlgo_ClampFinite(
        Arm_Control_Config.cascade_velocity_ki[axis],
        0.0f, ARM_CASCADE_VELOCITY_KI_MAX, 0.0f);
    velocity_limit = Arm_JointAlgo_ClampFinite(
        fabsf(Arm_Control_Config.cascade_velocity_limit[axis]),
        0.0f, V_MAX, 0.0f);
    integral_limit = Arm_JointAlgo_ClampFinite(
        fabsf(Arm_Control_Config.cascade_integral_limit[axis]),
        0.0f, T_MAX, 0.0f);
    torque_limit = Arm_JointAlgo_ClampFinite(
        fabsf(Arm_Control_Config.torque_limit[axis]),
        0.0f, T_MAX, 0.0f);

    output.target_velocity = Arm_JointAlgo_ClampFinite(
        position_kp * (target - position),
        -velocity_limit, velocity_limit, 0.0f);
    output.velocity_error = output.target_velocity - velocity;

    if (velocity_ki <= 0.0f) {
        s_cascade_integral_tau[axis] = 0.0f;
    }

    candidate_integral = s_cascade_integral_tau[axis] +
                         velocity_ki * output.velocity_error * dt_s;
    candidate_integral = Arm_JointAlgo_ClampFinite(
        candidate_integral, -integral_limit, integral_limit, 0.0f);
    proportional_tau = velocity_kp * output.velocity_error;
    candidate_tau = proportional_tau + candidate_integral;
    output.saturated = (candidate_tau > torque_limit || candidate_tau < -torque_limit) ? 1U : 0U;

    /* Conditional integration: do not accumulate further into saturation. */
    if ((candidate_tau <= torque_limit && candidate_tau >= -torque_limit) ||
        (candidate_tau > torque_limit && output.velocity_error < 0.0f) ||
        (candidate_tau < -torque_limit && output.velocity_error > 0.0f)) {
        s_cascade_integral_tau[axis] = candidate_integral;
    }

    output.integral_tau = s_cascade_integral_tau[axis];
    output.torque = Arm_JointAlgo_ClampFinite(
        proportional_tau + output.integral_tau,
        -torque_limit, torque_limit, 0.0f);
    return output;
}

float Arm_JointController_LimitTorque(uint8_t axis, float torque, uint8_t *saturated)
{
    if (saturated != NULL) *saturated = 0U;
    if (axis >= ARM_JOINT_COUNT) {
        if (saturated != NULL) *saturated = 1U;
        return 0.0f;
    }

    return Arm_JointAlgo_LimitTorque(torque,
                                     Arm_Control_Config.torque_limit[axis],
                                     T_MAX,
                                     saturated);
}
