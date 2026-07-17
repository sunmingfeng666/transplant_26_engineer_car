//
// Created by CaoKangqi on 2026/6/25.
//

#ifndef H7_FRAMEWORK_ROBOT_CONFIG_H
#define H7_FRAMEWORK_ROBOT_CONFIG_H

#include "BSP_TIM.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "LK_Motor.h"


typedef struct __attribute__((aligned(4))){
    DM_MOTOR_DATA_Typedef J1_8009;
    DM_MOTOR_DATA_Typedef J2_8009;
    DM_MOTOR_DATA_Typedef J3_4340;
    DM_MOTOR_DATA_Typedef J4_4340;
    DM_MOTOR_DATA_Typedef J5_4310;
    DM_MOTOR_DATA_Typedef J6_4310;
    DM_MOTOR_DATA_Typedef Terminal_3507;
} Arm_Motor_Group_t;

extern Arm_Motor_Group_t     arm_motors;

extern BSP_PWM_t imu_heater_pwm;
extern BSP_PWM_t picture_yaw_pwm;
extern BSP_PWM_t picture_pitch_pwm;

#endif //H7_FRAMEWORK_ROBOT_CONFIG_H
