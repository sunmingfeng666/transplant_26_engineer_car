#ifndef H7_FRAMEWORK_ENGINEER_LEADSCREW_CTRL_H
#define H7_FRAMEWORK_ENGINEER_LEADSCREW_CTRL_H

#include <stdint.h>
#include "Robot_Config.h"

typedef enum {
    ENGINEER_LEADSCREW_WAIT_FEEDBACK = 0,
    ENGINEER_LEADSCREW_TRACKING,
    ENGINEER_LEADSCREW_HOMING,
    ENGINEER_LEADSCREW_STOPPED,
    ENGINEER_LEADSCREW_FAULT
} Engineer_LeadScrew_State_e;

typedef struct {
    Engineer_LeadScrew_State_e state;
    int32_t position;
    uint8_t up_limit;
    uint8_t down_limit;
    uint8_t done;
    uint8_t homing_active;
    uint8_t homing_done;
    uint8_t fault;
} Engineer_LeadScrew_Status_t;

typedef struct {
    volatile uint8_t direct_enable;
    volatile float direct_speed;
    volatile int32_t encoder;
    volatile int16_t speed;
    volatile int16_t output;
    volatile uint8_t motor_online;
    volatile uint8_t up_limit;
    volatile uint8_t down_limit;
} Engineer_LeadScrew_Debug_t;

extern volatile Engineer_LeadScrew_Debug_t Engineer_LeadScrew_Debug;

uint8_t Engineer_LeadScrew_Init(void);
void Engineer_LeadScrew_Task(const Picture_Motor_Group_t *p_motor);
Engineer_LeadScrew_Status_t Engineer_LeadScrew_Get_Status(void);

// 供本板命令层写入的丝杠目标(编码器无限累加值域)。不走 B2B 协议。
void Engineer_LeadScrew_Set_Target(int32_t target);

// A 方案：丝杠电流由图传任务的 0x200 帧一并带出。图传发送前读取此值。
int16_t Engineer_LeadScrew_Get_Output(void);

#endif
