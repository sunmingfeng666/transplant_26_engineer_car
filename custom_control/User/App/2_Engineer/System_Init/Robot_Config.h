//
// Created by CaoKangqi on 2026/6/25.
//

#ifndef H7_FRAMEWORK_ENGINEER_ROBOT_CONFIG_H
#define H7_FRAMEWORK_ENGINEER_ROBOT_CONFIG_H

#include "BSP_TIM.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "LK_Motor.h"

typedef struct __attribute__((aligned(4))){
    DM_MOTOR_DATA_Typedef  J1_DM4310;
    DJI_MOTOR_DATA_Typedef J2_DJI2006;
    DJI_MOTOR_DATA_Typedef J3_DJI3508;
    DJI_MOTOR_DATA_Typedef J4_DJI3508;
    DJI_MOTOR_DATA_Typedef J5_DJI2006;
    DJI_MOTOR_DATA_Typedef J6_DJI2006;

    /* Target and output values stay global for live debugger watch. */
    float target_pos[6];
    float target_vel[6];
    int16_t output_current[6];
    uint8_t online_mask;
} Engineer_Custom_Motor_Group_t;

extern Engineer_Custom_Motor_Group_t engineer_custom_motors;

extern BSP_PWM_t imu_heater_pwm;
extern BSP_PWM_t trigger_pwm;

#endif //H7_FRAMEWORK_ENGINEER_ROBOT_CONFIG_H
