//
// Created by CaoKangqi on 2026/6/14.
//
#include "All_Task.h"
#include "Robot_Config.h"
#include "Controller_Transmit.h"
#include "Buzzer.h"
#include "DBUS.h"
#include "Message_Center.h"
#include "Power_CAP.h"
#include "Referee.h"
#include "Robot_Cmd.h"
#include "System_State.h"
#include "WS2812.h"
#include "System_Indicator.h"
#include "VT13.h"
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

    PubRegister("engineer_custom_motors", &engineer_custom_motors, sizeof(Engineer_Custom_Motor_Group_t));

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
void Motor_Task(void *argument)
{
    (void)argument;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(1);//绝对延时1ms

    System_State_Report(ID_CHASSIS, STATUS_RUN);
    System_State_Report(ID_GIMBAL, STATUS_RUN);
    System_State_Report(ID_SHOOT, STATUS_RUN);

    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);

        Controller_Transmit_Update();
    }
}


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
