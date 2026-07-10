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
    DJI_MOTOR_DATA_Typedef DJI_3508_Chassis[4];
    DJI_MOTOR_DATA_Typedef DJI_6020_Steer[4];
} Chassis_Motor_Group_t;

typedef struct __attribute__((aligned(4))){
    DJI_MOTOR_DATA_Typedef DJI_3508_Yaw;
    DM_MOTOR_DATA_Typedef DM4310_Pitch;
    DM_MOTOR_DATA_Typedef DM4310_Yaw;
} Gimbal_Motor_Group_t;

typedef struct __attribute__((aligned(4))){
    DJI_MOTOR_DATA_Typedef DJI_3508_Shoot_L;
    DJI_MOTOR_DATA_Typedef DJI_3508_Shoot_R;
    DJI_MOTOR_DATA_Typedef DJI_3508_Shoot_M;
    DM_MOTOR_DATA_Typedef DM4310_Feed;
    DJI_MOTOR_DATA_Typedef DJI_3508_Pull;
    DJI_MOTOR_DATA_Typedef DJI_3508_Travel;
    DJI_MOTOR_DATA_Typedef DJI_2006_bo;
} Shoot_Motor_Group_t;

// 工程机械臂电机组：6自由度关节 + 1末端夹爪，全部为达妙(DM)电机。
// 移植自旧臂主控 DM_H7_Master（2），关节空间控制（无正/逆运动学）。
// 关节-电机-总线-CAN ID 对应关系（⚠️ 需按实车接线核对）：
//   J1 肩   DM8009  0x01  FDCAN2  反馈ID 0x14  位置速度模式
//   J2 肩   DM8009  0x02  FDCAN2  反馈ID 0x15  位置速度模式
//   J3 roll DM4340  0x03  FDCAN2  反馈ID 0x16  位置速度模式
//   J4 pitch DM4340 0x04  FDCAN3  反馈ID 0x17  位置速度模式
//   J5 pitch DM4310 0x05  FDCAN3  反馈ID 0x18  位置速度模式
//   J6 roll DM4310  0x06  FDCAN3  反馈ID 0x19  位置速度模式
//   末端夹爪 DM3507  0x07  FDCAN3  反馈ID 0x1A  MIT力矩模式
typedef struct __attribute__((aligned(4))){
    DM_MOTOR_DATA_Typedef J1_8009;
    DM_MOTOR_DATA_Typedef J2_8009;
    DM_MOTOR_DATA_Typedef J3_4340;
    DM_MOTOR_DATA_Typedef J4_4340;
    DM_MOTOR_DATA_Typedef J5_4310;
    DM_MOTOR_DATA_Typedef J6_4310;
    DM_MOTOR_DATA_Typedef Terminal_3507;
} Arm_Motor_Group_t;

extern Chassis_Motor_Group_t chassis_motors;
extern Gimbal_Motor_Group_t  gimbal_motors;
extern Shoot_Motor_Group_t   shoot_motors;
extern Arm_Motor_Group_t     arm_motors;

extern BSP_PWM_t imu_heater_pwm;
extern BSP_PWM_t trigger_pwm;
extern BSP_PWM_t picture_yaw_pwm;
extern BSP_PWM_t picture_pitch_pwm;

#endif //H7_FRAMEWORK_ROBOT_CONFIG_H
