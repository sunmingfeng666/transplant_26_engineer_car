#ifndef H7_FRAMEWORK_ARM_JOINT_CONTROLLER_H
#define H7_FRAMEWORK_ARM_JOINT_CONTROLLER_H

#include <stdint.h>

#include "Arm_Joint_Algorithm.h"

#define ARM_JOINT_COUNT 6U

typedef enum {
    ARM_MODE_POSITION = 0,
    ARM_MODE_MIT,
    ARM_MODE_GRAVITY,
    ARM_MODE_GRAVITY_IMPEDANCE,
    ARM_MODE_CASCADE,
    ARM_MODE_DISABLED,
} Arm_Control_Mode_e;

typedef enum {
    ARM_STATE_WAIT_FEEDBACK = 0,
    ARM_STATE_POSITION_HOLD,
    ARM_STATE_MODE_RAMP,
    ARM_STATE_ACTIVE,
    ARM_STATE_DEGRADED,
    ARM_STATE_DISABLED,
} Arm_Control_State_e;

typedef struct {
    volatile uint8_t master_enable;
    volatile uint8_t axis_mode[ARM_JOINT_COUNT];
    volatile float gravity_scale[ARM_JOINT_COUNT];
    volatile float impedance_kp[ARM_JOINT_COUNT];
    volatile float impedance_ki[ARM_JOINT_COUNT];
    volatile float impedance_kd[ARM_JOINT_COUNT];
    volatile float impedance_i_limit[ARM_JOINT_COUNT];
    volatile float torque_limit[ARM_JOINT_COUNT];
    volatile float cascade_position_kp[ARM_JOINT_COUNT];
    volatile float cascade_velocity_kp[ARM_JOINT_COUNT];
    volatile float cascade_velocity_ki[ARM_JOINT_COUNT];
    volatile float cascade_velocity_limit[ARM_JOINT_COUNT];
    volatile float cascade_integral_limit[ARM_JOINT_COUNT];
    volatile float ramp_time_s;
    Arm_Gravity_Model_t gravity[ARM_JOINT_COUNT];
    /* 一键动作触发：0=无，1=展开,2=收回,3/4=自保护,5=存矿,6=取矿,7=机构复位。
       引擎执行完自动清零。abort 置 1 可随时中止当前动作。 */
    volatile uint8_t oneclick_request;
    volatile uint8_t oneclick_abort;
    volatile uint8_t oneclick_store_slot;
} Arm_Control_Config_t;

typedef struct {
    volatile uint8_t state;
    volatile uint8_t remote_online;
    volatile uint16_t online_mask;
    volatile uint16_t fault_mask;
    volatile uint16_t saturation_mask;
    volatile float target[ARM_JOINT_COUNT];
    volatile float position[ARM_JOINT_COUNT];
    volatile float position_error[ARM_JOINT_COUNT];
    volatile float velocity[ARM_JOINT_COUNT];
    volatile float cascade_target_velocity[ARM_JOINT_COUNT];
    volatile float cascade_velocity_error[ARM_JOINT_COUNT];
    volatile float cascade_integral_tau[ARM_JOINT_COUNT];
    volatile float gravity_tau[ARM_JOINT_COUNT];
    volatile float impedance_tau[ARM_JOINT_COUNT];
    volatile float command_tau[ARM_JOINT_COUNT];
    volatile float ramp[ARM_JOINT_COUNT];
    /* 一键动作遥测：active=是否执行中，id=当前动作编号，phase=内部阶段。 */
    volatile uint8_t oneclick_active;
    volatile uint8_t oneclick_id;
    volatile uint8_t oneclick_phase;
    volatile uint8_t oneclick_result;
    volatile uint8_t store_occupied_mask;
    volatile uint8_t clamp_close;
} Arm_Control_Debug_t;

typedef struct {
    float target_velocity;
    float velocity_error;
    float integral_tau;
    float torque;
    uint8_t saturated;
} Arm_Cascade_Output_t;

extern volatile Arm_Control_Config_t Arm_Control_Config;
extern volatile Arm_Control_Debug_t Arm_Control_Debug;

float Arm_JointController_Gravity(uint8_t axis, const float joint_position[ARM_JOINT_COUNT]);
float Arm_JointController_Impedance(uint8_t axis, float target, float position, float velocity);
Arm_Cascade_Output_t Arm_JointController_Cascade(uint8_t axis,
                                                 float target,
                                                 float position,
                                                 float velocity,
                                                 float dt_s);
void Arm_JointController_ResetCascade(uint8_t axis);
float Arm_JointController_LimitTorque(uint8_t axis, float torque, uint8_t *saturated);

#endif
