#ifndef H7_FRAMEWORK_ARM_JOINT_ALGORITHM_H
#define H7_FRAMEWORK_ARM_JOINT_ALGORITHM_H

#include <stdint.h>

typedef struct {
    volatile float a;
    volatile float b;
    volatile float c;
    volatile float d;
    volatile float e;
    volatile float f;
    volatile float g;
    volatile float h;
    volatile float i;
    volatile float j;
    volatile float k;
    volatile float l;
    volatile float m;
    volatile float min_rad;
    volatile float max_rad;
} Arm_Gravity_Model_t;

float Arm_JointAlgo_ClampFinite(float value, float min_value, float max_value, float fallback);
float Arm_JointAlgo_Gravity(uint8_t axis,
                            uint8_t joint_count,
                            const volatile Arm_Gravity_Model_t gravity[],
                            const volatile float gravity_scale[],
                            const float joint_position[]);
float Arm_JointAlgo_Impedance(float target,
                              float position,
                              float velocity,
                              float kp,
                              float kd,
                              float kp_max,
                              float kd_max);
float Arm_JointAlgo_LimitTorque(float torque,
                                float torque_limit,
                                float max_torque,
                                uint8_t *saturated);

#endif
