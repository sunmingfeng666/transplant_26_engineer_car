//
// Created by CaoKangqi on 2026/6/25.
//

#ifndef H7_FRAMEWORK_ROBOT_CONFIG_H
#define H7_FRAMEWORK_ROBOT_CONFIG_H

#include "BSP_TIM.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"

typedef struct __attribute__((aligned(4))){
    DJI_MOTOR_DATA_Typedef DJI_3508_Chassis[4];
} Chassis_Motor_Group_t;

typedef struct __attribute__((aligned(4))){
    DJI_MOTOR_DATA_Typedef DJI_2006_Lift;
    DJI_MOTOR_DATA_Typedef DJI_2006_Transverse;
    DJI_MOTOR_DATA_Typedef DJI_3508_LeadScrew;  // 登岛丝杠，与图传同在 CAN3、共用 0x200 帧
} Picture_Motor_Group_t;

typedef struct __attribute__((aligned(4))){
    DM_MOTOR_DATA_Typedef DM4310_Store;
} Store_Motor_Group_t;

extern Chassis_Motor_Group_t chassis_motors;
extern Picture_Motor_Group_t picture_motors;
extern Store_Motor_Group_t store_motors;

extern BSP_PWM_t imu_heater_pwm;

#endif //H7_FRAMEWORK_ROBOT_CONFIG_H
