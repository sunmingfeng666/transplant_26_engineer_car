//
// Created by CaoKangqi on 2026/6/14.
//
#include "All_Task.h"
#include "Robot_Config.h"
#include "Buzzer.h"
#include "DBUS.h"
#include "Message_Center.h"
#include "Power_CAP.h"
#include "Robot_Cmd.h"
#include "System_State.h"
#include "WS2812.h"
#include "System_Indicator.h"
#include "../../../Device/Host_Comm/Vofa.h"
#include "VT13.h"
#include "Arm_Ctrl.h"
#include "Arm_MatlabDebug.h"
//指令中心任务 200Hz
void Command_Task(void *argument)
{
    (void)argument;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(5);//绝对延时5ms
    PubRegister("dbus_data",  &DBUS,      sizeof(DBUS));
    PubRegister("vt13_data",  &VT13,      sizeof(VT13));
    PubRegister("imu_data",   &IMU_Data,  sizeof(IMU_Data));
    PubRegister("cap_data",   &cap,  sizeof(cap));

    PubRegister("chassis_motors", &chassis_motors, sizeof(Chassis_Motor_Group_t));
    PubRegister("gimbal_motors",  &gimbal_motors,  sizeof(Gimbal_Motor_Group_t));
    PubRegister("shoot_motors",   &shoot_motors,   sizeof(Shoot_Motor_Group_t));
    // 发布机械臂电机反馈快照，供 Motor_Task 订阅。
    PubRegister("arm_motors",     &arm_motors,     sizeof(Arm_Motor_Group_t));

    Robot_Cmd_Init();
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);

        Robot_Cmd_Update();
    }
}

//IMU姿态解算任务 1000Hz
void IMU_Task(void *argument)
{
    (void)argument;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(1);//绝对延时1ms

    static uint32_t INS_DWT_Count = 0; // DWT计数基准
    static float imu_period_s = 0.0f;
    INS_DWT_Count = DWT->CYCCNT;
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);

        imu_period_s = DWT_GetDeltaT(&INS_DWT_Count);
        IMU_Update_Task(&IMU_Data,imu_period_s);
    }
}

//运动控制任务 1000Hz
static IMU_Data_t imu ={0};
static Chassis_Motor_Group_t chassis_m = {0};
static Gimbal_Motor_Group_t gimbal_m = {0};
static Shoot_Motor_Group_t shoot_m = {0};
static Arm_Motor_Group_t arm_m = {0};
static DBUS_Typedef dbus_snap = {0};
void Motor_Task(void *argument)
{
    (void)argument;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(1);//绝对延时1ms

    Subscriber_t *imu_sub = NULL;
    Subscriber_t *c_motor_sub = NULL;
    Subscriber_t *g_motor_sub = NULL;
    Subscriber_t *s_motor_sub = NULL;
    Subscriber_t *a_motor_sub = NULL;
    Subscriber_t *dbus_sub = NULL;
    uint8_t vofa_divider = 0U;

    imu_sub = SubRegister("imu_data", sizeof(IMU_Data_t));
    c_motor_sub = SubRegister("chassis_motors", sizeof(Chassis_Motor_Group_t));
    g_motor_sub = SubRegister("gimbal_motors", sizeof(Gimbal_Motor_Group_t));
    s_motor_sub = SubRegister("shoot_motors", sizeof(Shoot_Motor_Group_t));
    a_motor_sub = SubRegister("arm_motors", sizeof(Arm_Motor_Group_t));
    dbus_sub = SubRegister("dbus_data", sizeof(DBUS_Typedef));

    // 机械臂控制初始化：默认位置保持，重力/阻抗需通过调试结构逐轴开启。
    Engineer_Arm_Init();

    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);

        if (imu_sub) SubGetMessage(imu_sub, &imu);
        if (c_motor_sub) SubGetMessage(c_motor_sub, &chassis_m);
        if (g_motor_sub) SubGetMessage(g_motor_sub, &gimbal_m);
        if (s_motor_sub)  SubGetMessage(s_motor_sub, &shoot_m);
        if (a_motor_sub) SubGetMessage(a_motor_sub, &arm_m);
        if (dbus_sub) SubGetMessage(dbus_sub, &dbus_snap);

        Engineer_Arm_Task(&arm_m, &dbus_snap, 0.001f);

        // ==========================================
        // UART7 遥测发送 @ 100Hz（board2 公用上位机口，常驻，不受联调开关控制）
        //   用途：VOFA 看波形、MATLAB 3D 显示。
        //   协议：JustFloat，115200 波特率；串口忙时底层直接丢帧，不阻塞 1kHz 电机控制任务。
        //   帧布局：
        //     ch0-5  = J1..J6 实际位置(rad)，MATLAB 取前 6 通道驱动 3D 模型。
        //     ch6-17 = J2/J4/J5 的 target/vel/gravity/command 调试量，VOFA 看波形。
        //     ch18   = 控制器状态（state）。
        //     ch19   = 联调链路在线标志（无联调编译开关时恒 0）。
        //   想看别的波形：直接改这一处 VOFA_JustFloat 的通道映射即可。
        // ==========================================
        if (++vofa_divider >= 10U) {
            vofa_divider = 0U;
#if ARM_MATLAB_DEBUG_ENABLE
            float link_online = (float)Arm_MatlabDebug_IsOnline();
#else
            float link_online = 0.0f;  /* 比赛固件无联调模块，恒 0 */
#endif
            VOFA_JustFloat(&huart7, 20,
                Arm_Control_Debug.position[0], Arm_Control_Debug.position[1],
                Arm_Control_Debug.position[2], Arm_Control_Debug.position[3],
                Arm_Control_Debug.position[4], Arm_Control_Debug.position[5],
                Arm_Control_Debug.target[1], Arm_Control_Debug.target[3], Arm_Control_Debug.target[4],
                Arm_Control_Debug.velocity[1], Arm_Control_Debug.velocity[3], Arm_Control_Debug.velocity[4],
                Arm_Control_Debug.gravity_tau[1], Arm_Control_Debug.gravity_tau[3], Arm_Control_Debug.gravity_tau[4],
                Arm_Control_Debug.command_tau[1], Arm_Control_Debug.command_tau[3], Arm_Control_Debug.command_tau[4],
                (float)Arm_Control_Debug.state, link_online);
        }
    }
}

//定时器中断
void MY_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    //定时器4 1000Hz
    if (htim->Instance == TIM4) {
        WS2812_Ticks();
        DWT_SysTimeUpdate();
        Offline_Monitor();
        System_State_Update();
        System_Indicator_Ticks();
    }
}
