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
#include "Chassis_Ctrl.h"
#include "Picture_Ctrl.h"
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

    // 发布硬件反馈快照，控制任务可通过 Message_Center 订阅，减少直接依赖全局变量。
    PubRegister("chassis_motors", &chassis_motors, sizeof(Chassis_Motor_Group_t));
    PubRegister("gimbal_motors",  &gimbal_motors,  sizeof(Gimbal_Motor_Group_t));
    PubRegister("shoot_motors",   &shoot_motors,   sizeof(Shoot_Motor_Group_t));
    PubRegister("picture_motors", &picture_motors, sizeof(Picture_Motor_Group_t));

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
static Picture_Motor_Group_t picture_m = {0};
void Motor_Task(void *argument)
{
    (void)argument;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(1);//绝对延时1ms

    Subscriber_t *imu_sub = NULL;
    Subscriber_t *c_motor_sub = NULL;
    Subscriber_t *g_motor_sub = NULL;
    Subscriber_t *s_motor_sub = NULL;
    Subscriber_t *p_motor_sub = NULL;

    imu_sub = SubRegister("imu_data", sizeof(IMU_Data_t));
    c_motor_sub = SubRegister("chassis_motors", sizeof(Chassis_Motor_Group_t));
    g_motor_sub = SubRegister("gimbal_motors", sizeof(Gimbal_Motor_Group_t));
    s_motor_sub = SubRegister("shoot_motors", sizeof(Shoot_Motor_Group_t));
    p_motor_sub = SubRegister("picture_motors", sizeof(Picture_Motor_Group_t));
    // 第一版移植这里只启用底盘闭环。
    Engineer_Chassis_Init();
    Engineer_Picture_Init();

    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);

        if (imu_sub) SubGetMessage(imu_sub, &imu);
        if (c_motor_sub) SubGetMessage(c_motor_sub, &chassis_m);
        if (g_motor_sub) SubGetMessage(g_motor_sub, &gimbal_m);
        if (s_motor_sub)  SubGetMessage(s_motor_sub, &shoot_m);
        if (p_motor_sub) SubGetMessage(p_motor_sub, &picture_m);

        // 底盘板电机输出路径：双板底盘命令 -> 四个 3508 电流。
        Engineer_Chassis_Task(&chassis_m);
        Engineer_Picture_Task(&picture_m);
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
