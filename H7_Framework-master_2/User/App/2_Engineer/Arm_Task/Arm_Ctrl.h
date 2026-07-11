#ifndef H7_FRAMEWORK_ENGINEER_ARM_CTRL_H
#define H7_FRAMEWORK_ENGINEER_ARM_CTRL_H

#include <stdint.h>

#include "Arm_JointController.h"
#include "DBUS.h"
#include "Robot_Config.h"

/*
 * 独立失能开关，沿用 MATLAB 联调分支的运行时开关风格。
 * 0 = 正常按 axis_mode[] 控制；非 0 = 6 个关节和末端夹爪全部下电。
 */
extern volatile uint8_t Arm_Disable_Enable;

uint8_t Engineer_Arm_Init(void);
void Engineer_Arm_Task(const Arm_Motor_Group_t *feedback,
                       const DBUS_Typedef *dbus,
                       float dt_s);

#endif
