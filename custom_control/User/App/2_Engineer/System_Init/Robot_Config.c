//
// Created by CaoKangqi on 2026/6/25.
//
#include "Offline_Detector.h"
#include "BSP_UART.h"
#include "BSP_FDCAN.h"
#include "All_define.h"
#include "DBUS.h"
#include "VT13.h"
#include "Referee.h"
#include "Robot_Config.h"
#include "Comm_DualBoard.h"
#include "Power_CAP.h"

Engineer_Custom_Motor_Group_t engineer_custom_motors;

BSP_PWM_t trigger_pwm = {&htim4, TIM_CHANNEL_2, PWM_CHANNEL_NORMAL};

UART_RX_NODE(&huart5, 18, DBUS_RX_DATA, NULL, 18, &DBUS, DBUS_Resolved);
OFFLINE_NODE(&DBUS.offline, DBUS_OFFLINE_TIME, GROUP_NONE);

UART_RX_NODE(&huart7, 21, VT13_RX_DATA, NULL, 21, &VT13, VT13_Resolved);
OFFLINE_NODE(&VT13.offline, DBUS_OFFLINE_TIME, GROUP_NONE);

UART_RX_NODE(&huart1, 0, Referee_Rx_Buf[0], Referee_Rx_Buf[1], REFEREE_RXFRAME_LENGTH, NULL, Referee_System_Frame_Update);
OFFLINE_NODE(&Referee.offline, REFEREE_OFFLINE_TIME, GROUP_NONE);


// 机械臂第一阶段：只注册 J1-J6 电机反馈，不接旧串口/自定义控制器链路。
CAN_RX_NODE(FDCAN2, 0x14,  &engineer_custom_motors.J1_DM4310,  DM_Standard_Resolve);
OFFLINE_NODE(&engineer_custom_motors.J1_DM4310.offline, MOTOR_OFFLINE_TIME, GROUP_NONE);

CAN_RX_NODE(FDCAN1, 0x202, &engineer_custom_motors.J2_DJI2006, DJI_Motor_Resolve);
OFFLINE_NODE(&engineer_custom_motors.J2_DJI2006.offline, MOTOR_OFFLINE_TIME, GROUP_NONE);

CAN_RX_NODE(FDCAN1, 0x203, &engineer_custom_motors.J3_DJI3508, DJI_Motor_Resolve);
OFFLINE_NODE(&engineer_custom_motors.J3_DJI3508.offline, MOTOR_OFFLINE_TIME, GROUP_NONE);

CAN_RX_NODE(FDCAN1, 0x204, &engineer_custom_motors.J4_DJI3508, DJI_Motor_Resolve);
OFFLINE_NODE(&engineer_custom_motors.J4_DJI3508.offline, MOTOR_OFFLINE_TIME, GROUP_NONE);

CAN_RX_NODE(FDCAN2, 0x205, &engineer_custom_motors.J5_DJI2006, DJI_Motor_Resolve);
OFFLINE_NODE(&engineer_custom_motors.J5_DJI2006.offline, MOTOR_OFFLINE_TIME, GROUP_NONE);

CAN_RX_NODE(FDCAN2, 0x206, &engineer_custom_motors.J6_DJI2006, DJI_Motor_Resolve);
OFFLINE_NODE(&engineer_custom_motors.J6_DJI2006.offline, MOTOR_OFFLINE_TIME, GROUP_NONE);
