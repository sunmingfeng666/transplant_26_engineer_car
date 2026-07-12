#ifndef H7_FRAMEWORK_ENGINEER_CHASSIS_CTRL_H
#define H7_FRAMEWORK_ENGINEER_CHASSIS_CTRL_H

#include <stdint.h>
#include "Robot_Config.h"
#include "Comm_DualBoard.h"

// 底盘运行状态快照，供 Feedback 模块统一上报（Chassis 本身不再直接发串口）。
typedef struct {
    DualBoard_Chassis_Feedback_Status_e status;
    uint8_t motor_online_bits;   // bit0~bit3 对应 0x201~0x204 四个 3508 在线
    int16_t error_code;
} Engineer_Chassis_Feedback_t;

// 底盘板控制入口。
// Init 配置 PID 和几何参数；Task 由 Motor_Task 以 1kHz 调用。
uint8_t Engineer_Chassis_Init(void);
void Engineer_Chassis_Task(const Chassis_Motor_Group_t *c_motor);
Engineer_Chassis_Feedback_t Engineer_Chassis_Get_Feedback(void);

#endif // H7_FRAMEWORK_ENGINEER_CHASSIS_CTRL_H
