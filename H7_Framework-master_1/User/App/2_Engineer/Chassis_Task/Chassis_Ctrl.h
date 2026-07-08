#ifndef H7_FRAMEWORK_ENGINEER_CHASSIS_CTRL_H
#define H7_FRAMEWORK_ENGINEER_CHASSIS_CTRL_H

#include <stdint.h>
#include "Robot_Config.h"

// 底盘板控制入口。
// Init 配置 PID 和几何参数；Task 由 Motor_Task 以 1kHz 调用。
uint8_t Engineer_Chassis_Init(void);
void Engineer_Chassis_Task(const Chassis_Motor_Group_t *c_motor);

#endif // H7_FRAMEWORK_ENGINEER_CHASSIS_CTRL_H
