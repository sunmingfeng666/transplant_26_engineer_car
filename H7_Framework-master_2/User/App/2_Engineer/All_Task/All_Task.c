//
// Created by CaoKangqi on 2026/6/14.
//
#include "All_Task.h"
#include "Robot_Config.h"
#include "Buzzer.h"
#include "DBUS.h"
#include "Message_Center.h"
#include "Power_CAP.h"
#include "Referee.h"
#include "Robot_Cmd.h"
#include "System_State.h"
#include "WS2812.h"
#include "System_Indicator.h"
#include "../../../Device/Host_Comm/Vofa.h"
#include "VT13.h"
#include "Arm_Ctrl.h"
//指令中心任务 200Hz
void Command_Task(void *argument)
{
    (void)argument;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(5);//绝对延时5ms
    PubRegister("dbus_data",  &DBUS,      sizeof(DBUS));
    PubRegister("vt13_data",  &VT13,      sizeof(VT13));
    PubRegister("referee_data",  &Referee,      sizeof(Referee_Data_t));
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
static VT13_Typedef vt13_snap = {0};
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
    Subscriber_t *vt13_sub = NULL;

    imu_sub = SubRegister("imu_data", sizeof(IMU_Data_t));
    c_motor_sub = SubRegister("chassis_motors", sizeof(Chassis_Motor_Group_t));
    g_motor_sub = SubRegister("gimbal_motors", sizeof(Gimbal_Motor_Group_t));
    s_motor_sub = SubRegister("shoot_motors", sizeof(Shoot_Motor_Group_t));
    a_motor_sub = SubRegister("arm_motors", sizeof(Arm_Motor_Group_t));
    dbus_sub = SubRegister("dbus_data", sizeof(DBUS_Typedef));
    vt13_sub = SubRegister("vt13_data", sizeof(VT13_Typedef));

    // 机械臂控制初始化（第一版：基础遥操作）。
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
        if (vt13_sub) SubGetMessage(vt13_sub, &vt13_snap);

        // 机械臂电机输出路径：DBUS/VT13 遥控 → 7 个达妙关节/夹爪。
        Engineer_Arm_Task(&arm_m, &dbus_snap, &vt13_snap);
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
        VOFA_JustFloat(NULL, 5, 0.0f, 1.0f,2.0f);
    }
}