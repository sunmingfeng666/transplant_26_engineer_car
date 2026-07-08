//
// 工程机械臂控制入口
// 移植自旧臂主控 DM_H7_Master（2）的 Move_Task / One_Click_Task。
// 第一阶段：仅基础遥操作（7电机使能、上电缓抬、关节限幅、DBUS/VT13手动控制）。
//
#ifndef H7_FRAMEWORK_ENGINEER_ARM_CTRL_H
#define H7_FRAMEWORK_ENGINEER_ARM_CTRL_H

#include <stdint.h>
#include "Robot_Config.h"
#include "DBUS.h"
#include "VT13.h"

// Init：配置各关节位置速度模式的速度上限、末端夹爪初态；由 Motor_Task 启动时调用一次。
// Task：由 Motor_Task 以 1kHz 调用，完成上电缓抬 → 读遥控输入 → 关节限幅 → 全在线才下发控制帧。
uint8_t Engineer_Arm_Init(void);
void Engineer_Arm_Task(const Arm_Motor_Group_t *a_motor, const DBUS_Typedef *dbus, const VT13_Typedef *vt13);

#endif // H7_FRAMEWORK_ENGINEER_ARM_CTRL_H
