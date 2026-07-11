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

Chassis_Motor_Group_t chassis_motors;
Gimbal_Motor_Group_t  gimbal_motors;
Shoot_Motor_Group_t   shoot_motors;
Picture_Motor_Group_t picture_motors;

BSP_PWM_t trigger_pwm = {&htim4, TIM_CHANNEL_2, PWM_CHANNEL_NORMAL};

// 保留新框架原有遥控接收：DBUS 使用 UART5，VT13 使用 UART7。
UART_RX_NODE(&huart5, 18, DBUS_RX_DATA, NULL, 18, &DBUS, DBUS_Resolved);
OFFLINE_NODE(&DBUS.offline, DBUS_OFFLINE_TIME, GROUP_NONE);

UART_RX_NODE(&huart7, 21, VT13_RX_DATA, NULL, 21, &VT13, VT13_Resolved);
OFFLINE_NODE(&VT13.offline, DBUS_OFFLINE_TIME, GROUP_NONE);

// 裁判系统固定接到底盘板 USART1：双缓冲 DMA 接收支持连续帧和跨回调断包。
UART_RX_NODE(&huart1, 0, Referee_Rx_Buf[0], Referee_Rx_Buf[1],
             REFEREE_RXFRAME_LENGTH, NULL, Referee_System_Frame_Update);
OFFLINE_NODE(&Referee.offline, REFEREE_OFFLINE_TIME, GROUP_NONE);

// 底盘板 USART10 接收遥控板 USART10 发来的固定长度底盘帧。
// 两个缓冲区放在 D2 RAM，因为 DMA 不能访问 H7 的所有内存区域。
static uint8_t DualBoard_Rx_Buf[2][DUALBOARD_ENGINEER_FRAME_LEN] __attribute__((section(".RAM_D2")));
UART_RX_NODE(&huart10, DUALBOARD_ENGINEER_FRAME_LEN, DualBoard_Rx_Buf[0], DualBoard_Rx_Buf[1],
             DUALBOARD_ENGINEER_FRAME_LEN, NULL, DualBoard_UART_Rx_Callback);

// 四个 3508 底盘电机，反馈 CAN ID 为 0x201~0x204，电流控制帧为 0x200。
CAN_RX_NODE(FDCAN1, 0x201, &chassis_motors.DJI_3508_Chassis[0], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_3508_Chassis[0].offline, MOTOR_OFFLINE_TIME, CHASSIS);
CAN_RX_NODE(FDCAN1, 0x202, &chassis_motors.DJI_3508_Chassis[1], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_3508_Chassis[1].offline, MOTOR_OFFLINE_TIME, CHASSIS);
CAN_RX_NODE(FDCAN1, 0x203, &chassis_motors.DJI_3508_Chassis[2], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_3508_Chassis[2].offline, MOTOR_OFFLINE_TIME, CHASSIS);
CAN_RX_NODE(FDCAN1, 0x204, &chassis_motors.DJI_3508_Chassis[3], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_3508_Chassis[3].offline, MOTOR_OFFLINE_TIME, CHASSIS);

CAN_RX_NODE(FDCAN3, 0x204, &picture_motors.DJI_2006_Lift, DJI_Motor_Resolve);
OFFLINE_NODE(&picture_motors.DJI_2006_Lift.offline, MOTOR_OFFLINE_TIME, GROUP_NONE);
CAN_RX_NODE(FDCAN3, 0x205, &picture_motors.DJI_2006_Transverse, DJI_Motor_Resolve);
OFFLINE_NODE(&picture_motors.DJI_2006_Transverse.offline, MOTOR_OFFLINE_TIME, GROUP_NONE);
