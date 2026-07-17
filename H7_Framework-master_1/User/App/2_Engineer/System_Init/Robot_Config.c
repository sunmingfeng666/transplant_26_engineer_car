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
#include "Power_Meter.h"

Chassis_Motor_Group_t chassis_motors;
Picture_Motor_Group_t picture_motors;
Store_Motor_Group_t store_motors;

// DBUS 与 VT13 都由机械臂板接收；底盘板只解析 USART10 转发的两份原始帧。

// 裁判系统固定接到底盘板 USART1：双缓冲 DMA 接收支持连续帧和跨回调断包。
UART_RX_NODE(&huart1, 0, Referee_Rx_Buf[0], Referee_Rx_Buf[1],
             REFEREE_RXFRAME_LENGTH, NULL, Referee_System_Frame_Update);
OFFLINE_NODE(&Referee.offline, REFEREE_OFFLINE_TIME, GROUP_NONE);

// 底盘板 USART10 接收遥控板发来的 V3 固定长度原始遥控帧。
// 两个缓冲区放在 D2 RAM，因为 DMA 不能访问 H7 的所有内存区域。
static uint8_t DualBoard_Rx_Buf[2][DUALBOARD_REMOTE_FRAME_LEN] __attribute__((section(".RAM_D2")));
UART_RX_NODE(&huart10, DUALBOARD_REMOTE_FRAME_LEN, DualBoard_Rx_Buf[0], DualBoard_Rx_Buf[1],
             DUALBOARD_REMOTE_FRAME_LEN, NULL, DualBoard_UART_Rx_Callback);

// 四个 3508 底盘电机，反馈 CAN ID 为 0x201~0x204，电流控制帧为 0x200。
CAN_RX_NODE(FDCAN1, 0x201, &chassis_motors.DJI_3508_Chassis[0], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_3508_Chassis[0].offline, MOTOR_OFFLINE_TIME, CHASSIS);
CAN_RX_NODE(FDCAN1, 0x202, &chassis_motors.DJI_3508_Chassis[1], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_3508_Chassis[1].offline, MOTOR_OFFLINE_TIME, CHASSIS);
CAN_RX_NODE(FDCAN1, 0x203, &chassis_motors.DJI_3508_Chassis[2], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_3508_Chassis[2].offline, MOTOR_OFFLINE_TIME, CHASSIS);
CAN_RX_NODE(FDCAN1, 0x204, &chassis_motors.DJI_3508_Chassis[3], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_3508_Chassis[3].offline, MOTOR_OFFLINE_TIME, CHASSIS);

// 老车独立功率计位于底盘板 FDCAN1，使用标准帧 0x605。
CAN_RX_NODE(FDCAN1, POWER_METER_CAN_ID, &power_meter, Power_Meter_Rx);
OFFLINE_NODE(&power_meter.offline, POWER_METER_OFFLINE_TIME, GROUP_NONE);

// 沿用老车 CAN3 接线：ID3=图传抬升、ID4=登岛丝杠、ID5=图传横移。
CAN_RX_NODE(FDCAN3, 0x203, &picture_motors.DJI_2006_Lift, DJI_Motor_Resolve);
OFFLINE_NODE(&picture_motors.DJI_2006_Lift.offline, MOTOR_OFFLINE_TIME, GROUP_NONE);
CAN_RX_NODE(FDCAN3, 0x205, &picture_motors.DJI_2006_Transverse, DJI_Motor_Resolve);
OFFLINE_NODE(&picture_motors.DJI_2006_Transverse.offline, MOTOR_OFFLINE_TIME, GROUP_NONE);

// 登岛丝杠 DJI 3508：电调 ID=4，反馈 0x204，使用 0x200 电流帧第 4 槽 n4。
#ifndef LEADSCREW_RX_ID
#define LEADSCREW_RX_ID 0x204
#endif
CAN_RX_NODE(FDCAN3, LEADSCREW_RX_ID, &picture_motors.DJI_3508_LeadScrew, DJI_Motor_Resolve);
OFFLINE_NODE(&picture_motors.DJI_3508_LeadScrew.offline, MOTOR_OFFLINE_TIME, GROUP_NONE);

// 存矿 DM4310 使用位置速度模式：命令基 ID 0x03，反馈 ID 0x2C。
CAN_RX_NODE(FDCAN2, 0x2C, &store_motors.DM4310_Store, DM_Standard_Resolve);
OFFLINE_NODE(&store_motors.DM4310_Store.offline, MOTOR_OFFLINE_TIME, GROUP_NONE);
