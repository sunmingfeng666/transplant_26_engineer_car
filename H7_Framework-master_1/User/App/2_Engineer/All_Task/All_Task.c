//
// Created by CaoKangqi on 2026/6/14.
//
#include "All_Task.h"
#include "Robot_Config.h"
#include "Buzzer.h"
#include "DBUS.h"
#include "Message_Center.h"
#include "Power_CAP.h"
#include "Power_Meter.h"
#include "Referee.h"
#include "Robot_Cmd.h"
#include "System_State.h"
#include "WS2812.h"
#include "System_Indicator.h"
#include "VT13.h"
#include "Chassis_Ctrl.h"
#include "Picture_Ctrl.h"
#include "LeadScrew_Ctrl.h"
#include "Store_Ctrl.h"
#include "Engineer_Feedback.h"
#include "Comm_DualBoard.h"
#include "Vofa.h"
#include "Custom_Controller_Link.h"

// 板1执行机构调试快照：只用于集中观察，不参与实际控制。
typedef struct {
    B2B_Chassis_Cmd_t chassis;
    B2B_Picture_Cmd_t picture;
    Engineer_LeadScrew_Status_t lead_screw;
    int16_t lead_screw_output;
    Power_Meter_t power_meter;  // 板1 FDCAN1/0x605 独立功率计快照
} B2B_Executor_Debug_t;

volatile B2B_Executor_Debug_t B2B_Executor_Debug = {0};

static void B2B_Executor_Debug_Update(void)
{
    B2B_Executor_Debug_t snapshot = {0};

    snapshot.chassis = B2B_Chassis_Cmd;
    snapshot.picture = B2B_Picture_Cmd;
    snapshot.lead_screw = Engineer_LeadScrew_Get_Status();
    snapshot.lead_screw_output = Engineer_LeadScrew_Get_Output();
    snapshot.power_meter = power_meter;
    B2B_Executor_Debug = snapshot;
}
//指令中心任务 200Hz
void Command_Task(void *argument)
{
    (void)argument;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(5);//绝对延时5ms
    PubRegister("dbus_data",  &DBUS,      sizeof(DBUS));
    PubRegister("vt13_data",  &VT13,      sizeof(VT13));
    PubRegister("referee_data",  &Referee,      sizeof(Referee_Data_t));
    PubRegister("cap_data",   &cap,  sizeof(cap));

    // 发布硬件反馈快照，控制任务可通过 Message_Center 订阅，减少直接依赖全局变量。
    PubRegister("chassis_motors", &chassis_motors, sizeof(Chassis_Motor_Group_t));
    PubRegister("picture_motors", &picture_motors, sizeof(Picture_Motor_Group_t));
    PubRegister("store_motors", &store_motors, sizeof(Store_Motor_Group_t));

    Robot_Cmd_Init();
    Custom_Controller_Link_Init();
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);

        Robot_Cmd_Update();
        Custom_Controller_Link_Update();
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
static Chassis_Motor_Group_t chassis_m = {0};
static Picture_Motor_Group_t picture_m = {0};
static Store_Motor_Group_t store_m = {0};
static uint8_t vofa_divider = 0U;

void Motor_Task(void *argument)
{
    (void)argument;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(1);//绝对延时1ms

    Subscriber_t *c_motor_sub = NULL;
    Subscriber_t *p_motor_sub = NULL;
    Subscriber_t *store_motor_sub = NULL;

    c_motor_sub = SubRegister("chassis_motors", sizeof(Chassis_Motor_Group_t));
    p_motor_sub = SubRegister("picture_motors", sizeof(Picture_Motor_Group_t));
    store_motor_sub = SubRegister("store_motors", sizeof(Store_Motor_Group_t));
    Engineer_Chassis_Init();
    Engineer_Picture_Init();
    Engineer_LeadScrew_Init();
    Engineer_Store_Init();
    Engineer_Feedback_Init();

    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);

        if (c_motor_sub) SubGetMessage(c_motor_sub, &chassis_m);
        if (p_motor_sub) SubGetMessage(p_motor_sub, &picture_m);
        if (store_motor_sub) SubGetMessage(store_motor_sub, &store_m);

        // 底盘板电机输出路径：双板底盘命令 -> 四个 3508 电流。
        Engineer_Chassis_Task(&chassis_m);
        // 丝杠须在图传之前算好电流：二者共用 CAN3 的 0x200 帧，由图传任务统一发送。
        Engineer_LeadScrew_Task(&picture_m);
        Engineer_Picture_Task(&picture_m);
        Engineer_Store_Task(&store_m);
        Engineer_Feedback_Task(&chassis_m, &picture_m, &store_m);

        // 每个 1 ms 控制周期结束后刷新一次，Ozone 只需观察这一个全局变量。
        B2B_Executor_Debug_Update();

        if (++vofa_divider >= 10U)
        {
            vofa_divider = 0U;

            // 目标/实际交替发送，VOFA 中每相邻两条曲线对应同一个底盘轮。
            VOFA_JustFloat(&huart7, 8,
                           Engineer_Chassis_Speed[0].Target_rpm,
                           Engineer_Chassis_Speed[0].Speed_now,
                           Engineer_Chassis_Speed[1].Target_rpm,
                           Engineer_Chassis_Speed[1].Speed_now,
                           Engineer_Chassis_Speed[2].Target_rpm,
                           Engineer_Chassis_Speed[2].Speed_now,
                           Engineer_Chassis_Speed[3].Target_rpm,
                           Engineer_Chassis_Speed[3].Speed_now);
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
