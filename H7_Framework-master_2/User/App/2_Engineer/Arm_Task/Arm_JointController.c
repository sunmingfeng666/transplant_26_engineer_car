#include "Arm_JointController.h"

#include "DM_Motor.h"

#define ARM_AXIS_J2 1U
#define ARM_AXIS_J4 3U
#define ARM_AXIS_J5 4U

volatile Arm_Control_Config_t Arm_Control_Config = {
    .master_enable = 1U,
    .axis_mode = {
        ARM_MODE_POSITION, ARM_MODE_GRAVITY_IMPEDANCE, ARM_MODE_MIT,
        ARM_MODE_GRAVITY_IMPEDANCE, ARM_MODE_POSITION, ARM_MODE_MIT,
    },
    .gravity_scale = {0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f},
    .impedance_kp = {0.0f, 12.0f, 4.0f, 6.0f, 0.0f, 3.0f},
    .impedance_ki = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .impedance_kd = {0.0f, 4.0f, 0.2f, 0.25f, 0.0f, 0.15f},
    .impedance_i_limit = {0.0f, 0.5f, 0.5f, 0.5f, 0.0f, 0.5f},
    .torque_limit = {20.0f, 20.0f, 20.0f, 20.0f, 20.0f, 20.0f},
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

float Arm_JointController_Gravity(uint8_t axis, float q2, float q4, float q5)
{
    return Arm_JointAlgo_Gravity(axis,
                                 ARM_JOINT_COUNT,
                                 Arm_Control_Config.gravity,
                                 Arm_Control_Config.gravity_scale,
                                 q2,
                                 q4,
                                 q5);
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
