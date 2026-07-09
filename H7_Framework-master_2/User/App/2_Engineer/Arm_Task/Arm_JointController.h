#ifndef H7_FRAMEWORK_ARM_JOINT_CONTROLLER_H
#define H7_FRAMEWORK_ARM_JOINT_CONTROLLER_H

#include <stdint.h>

#define ARM_JOINT_COUNT 6U

typedef enum {
    ARM_MODE_POSITION = 0,
    ARM_MODE_GRAVITY,
    ARM_MODE_GRAVITY_IMPEDANCE,
    ARM_MODE_DISABLED,  // 失能模式
} Arm_Control_Mode_e;

typedef enum {
    ARM_STATE_WAIT_FEEDBACK = 0,
    ARM_STATE_POSITION_HOLD,
    ARM_STATE_MODE_RAMP,
    ARM_STATE_ACTIVE,
    ARM_STATE_DEGRADED,
    ARM_STATE_DISABLED,  // 所有电机失能
} Arm_Control_State_e;

typedef struct {
    volatile float a;
    volatile float b;
    volatile float c;
    volatile float d;
    volatile float e;
    volatile float f;
    volatile float g;
    volatile float min_rad;
    volatile float max_rad;
} Arm_Gravity_Model_t;

typedef struct {
    /* 总开关默认关闭；调试时先设置单轴模式，再打开总开关。 */
    volatile uint8_t master_enable;
    volatile uint8_t axis_mode[ARM_JOINT_COUNT];
    volatile float gravity_scale[ARM_JOINT_COUNT];
    volatile float impedance_kp[ARM_JOINT_COUNT];
    volatile float impedance_kd[ARM_JOINT_COUNT];
    volatile float torque_limit[ARM_JOINT_COUNT];
    volatile float ramp_time_s;
    Arm_Gravity_Model_t gravity[ARM_JOINT_COUNT];
} Arm_Control_Config_t;

typedef struct {
    volatile uint8_t state;
    volatile uint8_t remote_online;
    volatile uint16_t online_mask;
    volatile uint16_t fault_mask;
    volatile uint16_t saturation_mask;
    volatile float target[ARM_JOINT_COUNT];
    volatile float position[ARM_JOINT_COUNT];
    volatile float velocity[ARM_JOINT_COUNT];
    volatile float gravity_tau[ARM_JOINT_COUNT];
    volatile float impedance_tau[ARM_JOINT_COUNT];
    volatile float command_tau[ARM_JOINT_COUNT];
    volatile float ramp[ARM_JOINT_COUNT];
} Arm_Control_Debug_t;

extern volatile Arm_Control_Config_t Arm_Control_Config;
extern volatile Arm_Control_Debug_t Arm_Control_Debug;

float Arm_JointController_Gravity(uint8_t axis, float q2, float q4, float q5);
float Arm_JointController_Impedance(uint8_t axis, float target, float position, float velocity);
float Arm_JointController_LimitTorque(uint8_t axis, float torque, uint8_t *saturated);

#endif
