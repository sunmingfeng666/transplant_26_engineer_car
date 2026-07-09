#include "Arm_JointController.h"

#include <math.h>

#include "DM_Motor.h"

#define ARM_AXIS_J2 1U
#define ARM_AXIS_J4 3U
#define ARM_AXIS_J5 4U

volatile Arm_Control_Config_t Arm_Control_Config = {
    .master_enable = 0U,
    .axis_mode = {
        ARM_MODE_POSITION, ARM_MODE_POSITION, ARM_MODE_POSITION,
        ARM_MODE_POSITION, ARM_MODE_POSITION, ARM_MODE_POSITION,
    },
    .gravity_scale = {0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f},
    .impedance_kp = {0.0f, 12.0f, 0.0f, 6.0f, 5.0f, 0.0f},
    .impedance_kd = {0.0f, 4.0f, 0.0f, 0.25f, 0.18f, 0.0f},
    .torque_limit = {0.0f, 4.0f, 0.0f, 4.0f, 2.5f, 0.0f},
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
};

volatile Arm_Control_Debug_t Arm_Control_Debug = {
    .state = ARM_STATE_WAIT_FEEDBACK,
};

static float Arm_ClampFinite(float value, float min_value, float max_value, float fallback)
{
    if (!isfinite(value)) return fallback;
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

float Arm_JointController_Gravity(uint8_t axis, float q2, float q4, float q5)
{
    const volatile Arm_Gravity_Model_t *model;
    float scale;

    if (axis >= ARM_JOINT_COUNT) return 0.0f;
    model = &Arm_Control_Config.gravity[axis];
    scale = Arm_ClampFinite(Arm_Control_Config.gravity_scale[axis], 0.0f, 2.0f, 0.0f);

    if (axis == ARM_AXIS_J2) {
        q2 = Arm_ClampFinite(q2, model->min_rad, model->max_rad, 0.0f);
        q4 = Arm_ClampFinite(q4,
                             Arm_Control_Config.gravity[ARM_AXIS_J4].min_rad,
                             Arm_Control_Config.gravity[ARM_AXIS_J4].max_rad,
                             0.0f);
        return scale * (model->a * sinf(q2) +
                        model->b * cosf(q2) +
                        model->d * sinf(q2 + q4) +
                        model->e * cosf(q2 + q4) +
                        model->c);
    }

    if (axis == ARM_AXIS_J4) {
        q2 = Arm_ClampFinite(q2,
                             Arm_Control_Config.gravity[ARM_AXIS_J2].min_rad,
                             Arm_Control_Config.gravity[ARM_AXIS_J2].max_rad,
                             0.0f);
        q4 = Arm_ClampFinite(q4, model->min_rad, model->max_rad, 0.0f);
        return scale * (model->a * sinf(q2) +
                        model->b * cosf(q2) +
                        model->d * sinf(q4) +
                        model->e * cosf(q4) +
                        model->f * sinf(q2 + q4) +
                        model->g * cosf(q2 + q4) +
                        model->c);
    }

    if (axis == ARM_AXIS_J5) {
        q4 = Arm_ClampFinite(q4,
                             Arm_Control_Config.gravity[ARM_AXIS_J4].min_rad,
                             Arm_Control_Config.gravity[ARM_AXIS_J4].max_rad,
                             0.0f);
        q5 = Arm_ClampFinite(q5, model->min_rad, model->max_rad, 0.0f);
        return scale * (model->a * sinf(q5) +
                        model->b * cosf(q5) +
                        model->d * sinf(q4 + q5) +
                        model->e * cosf(q4 + q5) +
                        model->c);
    }

    return 0.0f;
}

float Arm_JointController_Impedance(uint8_t axis, float target, float position, float velocity)
{
    float kp;
    float kd;

    if (axis >= ARM_JOINT_COUNT ||
        !isfinite(target) || !isfinite(position) || !isfinite(velocity)) {
        return 0.0f;
    }

    kp = Arm_ClampFinite(Arm_Control_Config.impedance_kp[axis], 0.0f, KP_MAX, 0.0f);
    kd = Arm_ClampFinite(Arm_Control_Config.impedance_kd[axis], 0.0f, KD_MAX, 0.0f);
    return kp * (target - position) - kd * velocity;
}

float Arm_JointController_LimitTorque(uint8_t axis, float torque, uint8_t *saturated)
{
    float limit;

    if (saturated != NULL) *saturated = 0U;
    if (axis >= ARM_JOINT_COUNT || !isfinite(torque)) {
        if (saturated != NULL) *saturated = 1U;
        return 0.0f;
    }

    limit = Arm_ClampFinite(fabsf(Arm_Control_Config.torque_limit[axis]), 0.0f, T_MAX, 0.0f);
    if (torque > limit) {
        if (saturated != NULL) *saturated = 1U;
        return limit;
    }
    if (torque < -limit) {
        if (saturated != NULL) *saturated = 1U;
        return -limit;
    }
    return torque;
}
