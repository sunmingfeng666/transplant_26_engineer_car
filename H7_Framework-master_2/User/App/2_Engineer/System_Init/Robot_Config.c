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
#include "Arm_MatlabDebug.h"

Chassis_Motor_Group_t chassis_motors;
Gimbal_Motor_Group_t  gimbal_motors;
Shoot_Motor_Group_t   shoot_motors;
Arm_Motor_Group_t     arm_motors;

BSP_PWM_t trigger_pwm = {&htim4, TIM_CHANNEL_2, PWM_CHANNEL_NORMAL};
BSP_PWM_t picture_yaw_pwm = {&htim2, TIM_CHANNEL_1, PWM_CHANNEL_NORMAL};
BSP_PWM_t picture_pitch_pwm = {&htim2, TIM_CHANNEL_3, PWM_CHANNEL_NORMAL};

UART_RX_NODE(&huart5, 18, DBUS_RX_DATA, NULL, 18, &DBUS, DBUS_Resolved);
OFFLINE_NODE(&DBUS.offline, DBUS_OFFLINE_TIME, GROUP_NONE);

// ==========================================
// UART7 公用上位机口（board2）
//   发送：常驻，VOFA 波形/MATLAB 遥测共用（见 All_Task.c），不受联调开关控制。
//   接收：仅当机械臂 MATLAB 联调编译开关打开时，才注册接收解析（MATLAB 下发关节角）。
// ==========================================
#if ARM_MATLAB_DEBUG_ENABLE
// 变长 JustFloat 帧：expected_size=0 跳过定长校验，靠帧尾切分。
// 缓冲区放 D2 RAM，H7 的 DMA1 无法访问 DTCM。
static uint8_t Arm_MatlabDebug_Rx_Buf[2][ARM_MATLAB_DEBUG_RX_SIZE] __attribute__((section(".RAM_D2")));
UART_RX_NODE(&huart7, 0, Arm_MatlabDebug_Rx_Buf[0], Arm_MatlabDebug_Rx_Buf[1],
             ARM_MATLAB_DEBUG_RX_SIZE, NULL, Arm_MatlabDebug_Resolve);
#endif

// 遥控板 USART10 接收底盘板 USART10 回传的固定长度状态反馈帧。
// 两个缓冲区放在 D2 RAM，因为 DMA 不能访问 H7 的所有内存区域。
static uint8_t DualBoard_Rx_Buf[2][DUALBOARD_CHASSIS_FRAME_LEN] __attribute__((section(".RAM_D2")));
UART_RX_NODE(&huart10, DUALBOARD_CHASSIS_FRAME_LEN, DualBoard_Rx_Buf[0], DualBoard_Rx_Buf[1],
             DUALBOARD_CHASSIS_FRAME_LEN, NULL, DualBoard_UART_Rx_Callback);

UART_RX_NODE(&huart1, 0, Referee_Rx_Buf[0], Referee_Rx_Buf[1], REFEREE_RXFRAME_LENGTH, NULL, Referee_System_Frame_Update);
OFFLINE_NODE(&Referee.offline, REFEREE_OFFLINE_TIME, GROUP_NONE);

CAN_RX_NODE(FDCAN1, 0x201, &chassis_motors.DJI_3508_Chassis[0], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_3508_Chassis[0].offline, MOTOR_OFFLINE_TIME, CHASSIS);

// ================= 工程机械臂 7 个达妙电机自动注册 =================
// ⚠️ 反馈 CAN ID 与总线分配沿用旧臂主控 DM_H7_Master（2），需按实车接线核对。
// 达妙标准反馈帧统一用 DM_Standard_Resolve 解析（解算 pos/vel/tor/温度并喂离线检测）。
// J1-J3 挂在 FDCAN2，反馈 ID 0x14/0x15/0x16
CAN_RX_NODE(FDCAN2, 0x14, &arm_motors.J1_8009, DM_Standard_Resolve);
OFFLINE_NODE(&arm_motors.J1_8009.offline, MOTOR_OFFLINE_TIME, GROUP_NONE);
CAN_RX_NODE(FDCAN2, 0x15, &arm_motors.J2_8009, DM_Standard_Resolve);
OFFLINE_NODE(&arm_motors.J2_8009.offline, MOTOR_OFFLINE_TIME, GROUP_NONE);
CAN_RX_NODE(FDCAN2, 0x16, &arm_motors.J3_4340, DM_Standard_Resolve);
OFFLINE_NODE(&arm_motors.J3_4340.offline, MOTOR_OFFLINE_TIME, GROUP_NONE);
// J4-J6 + 末端夹爪挂在 FDCAN3，反馈 ID 0x17/0x18/0x19/0x1A
CAN_RX_NODE(FDCAN3, 0x17, &arm_motors.J4_4340, DM_Standard_Resolve);
OFFLINE_NODE(&arm_motors.J4_4340.offline, MOTOR_OFFLINE_TIME, GROUP_NONE);
CAN_RX_NODE(FDCAN3, 0x18, &arm_motors.J5_4310, DM_Standard_Resolve);
OFFLINE_NODE(&arm_motors.J5_4310.offline, MOTOR_OFFLINE_TIME, GROUP_NONE);
CAN_RX_NODE(FDCAN3, 0x19, &arm_motors.J6_4310, DM_Standard_Resolve);
OFFLINE_NODE(&arm_motors.J6_4310.offline, MOTOR_OFFLINE_TIME, GROUP_NONE);
CAN_RX_NODE(FDCAN3, 0x1A, &arm_motors.Terminal_3507, DM_Standard_Resolve);
OFFLINE_NODE(&arm_motors.Terminal_3507.offline, MOTOR_OFFLINE_TIME, GROUP_NONE);
