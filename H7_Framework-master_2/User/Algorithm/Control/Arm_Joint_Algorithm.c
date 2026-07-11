#include "Arm_Joint_Algorithm.h"

#include <math.h>
#include <stddef.h>

#define ARM_JOINT_ALGO_AXIS_J2 1U
#define ARM_JOINT_ALGO_AXIS_J4 3U
#define ARM_JOINT_ALGO_AXIS_J5 4U

float Arm_JointAlgo_ClampFinite(float value, float min_value, float max_value, float fallback)
{
    if (!isfinite(value)) return fallback;
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

float Arm_JointAlgo_Gravity(uint8_t axis,
                            uint8_t joint_count,
                            const volatile Arm_Gravity_Model_t gravity[],
                            const volatile float gravity_scale[],
                            float q2,
                            float q4,
                            float q5)
{
    const volatile Arm_Gravity_Model_t *model;
    float scale;

    if (gravity == NULL || gravity_scale == NULL || axis >= joint_count) return 0.0f;
    model = &gravity[axis];
    scale = Arm_JointAlgo_ClampFinite(gravity_scale[axis], 0.0f, 2.0f, 0.0f);

    if (axis == ARM_JOINT_ALGO_AXIS_J2) {
        if (joint_count <= ARM_JOINT_ALGO_AXIS_J4) return 0.0f;
        q2 = Arm_JointAlgo_ClampFinite(q2, model->min_rad, model->max_rad, 0.0f);
        q4 = Arm_JointAlgo_ClampFinite(q4,
                                       gravity[ARM_JOINT_ALGO_AXIS_J4].min_rad,
                                       gravity[ARM_JOINT_ALGO_AXIS_J4].max_rad,
                                       0.0f);
        return scale * (model->a * sinf(q2) +
                        model->b * cosf(q2) +
                        model->d * sinf(q2 + q4) +
                        model->e * cosf(q2 + q4) +
                        model->c);
    }

    if (axis == ARM_JOINT_ALGO_AXIS_J4) {
        if (joint_count <= ARM_JOINT_ALGO_AXIS_J4) return 0.0f;
        q2 = Arm_JointAlgo_ClampFinite(q2,
                                       gravity[ARM_JOINT_ALGO_AXIS_J2].min_rad,
                                       gravity[ARM_JOINT_ALGO_AXIS_J2].max_rad,
                                       0.0f);
        q4 = Arm_JointAlgo_ClampFinite(q4, model->min_rad, model->max_rad, 0.0f);
        return scale * (model->a * sinf(q2) +
                        model->b * cosf(q2) +
                        model->d * sinf(q4) +
                        model->e * cosf(q4) +
                        model->f * sinf(q2 + q4) +
                        model->g * cosf(q2 + q4) +
                        model->c);
    }

    if (axis == ARM_JOINT_ALGO_AXIS_J5) {
        if (joint_count <= ARM_JOINT_ALGO_AXIS_J5) return 0.0f;
        q4 = Arm_JointAlgo_ClampFinite(q4,
                                       gravity[ARM_JOINT_ALGO_AXIS_J4].min_rad,
                                       gravity[ARM_JOINT_ALGO_AXIS_J4].max_rad,
                                       0.0f);
        q5 = Arm_JointAlgo_ClampFinite(q5, model->min_rad, model->max_rad, 0.0f);
        return scale * (model->a * sinf(q5) +
                        model->b * cosf(q5) +
                        model->d * sinf(q4 + q5) +
                        model->e * cosf(q4 + q5) +
                        model->c);
    }

    return 0.0f;
}

float Arm_JointAlgo_Impedance(float target,
                              float position,
                              float velocity,
                              float kp,
                              float kd,
                              float kp_max,
                              float kd_max)
{
    if (!isfinite(target) || !isfinite(position) || !isfinite(velocity)) {
        return 0.0f;
    }

    kp = Arm_JointAlgo_ClampFinite(kp, 0.0f, kp_max, 0.0f);
    kd = Arm_JointAlgo_ClampFinite(kd, 0.0f, kd_max, 0.0f);
    return kp * (target - position) - kd * velocity;
}

float Arm_JointAlgo_LimitTorque(float torque,
                                float torque_limit,
                                float max_torque,
                                uint8_t *saturated)
{
    float limit;

    if (saturated != NULL) *saturated = 0U;
    if (!isfinite(torque)) {
        if (saturated != NULL) *saturated = 1U;
        return 0.0f;
    }

    if (!isfinite(max_torque) || max_torque < 0.0f) {
        max_torque = 0.0f;
    }
    limit = Arm_JointAlgo_ClampFinite(fabsf(torque_limit), 0.0f, max_torque, 0.0f);
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
