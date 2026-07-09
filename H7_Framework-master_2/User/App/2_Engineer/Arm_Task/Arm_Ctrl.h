#ifndef H7_FRAMEWORK_ENGINEER_ARM_CTRL_H
#define H7_FRAMEWORK_ENGINEER_ARM_CTRL_H

#include <stdint.h>

#include "Arm_JointController.h"
#include "DBUS.h"
#include "Robot_Config.h"

/*
 * 一键失能运行时开关（仿 MATLAB 联调开关 Arm_MatlabDebug_Enable 的独立开关风格）。
 *   0 = 正常控制；非 0 = 整臂失能（6 关节 + 末端夹爪全部下电，不发任何力矩/位置命令）。
 * 默认 0。取代旧的 master_enable==0xFF 魔数触发，与 master_enable 的模式选择解耦。
 * 置 1 即失能，置 0 恢复；恢复后各轴按 Arm_RequestedMode 重新配置模式并从当前位置续接。
 */
extern volatile uint8_t Arm_Disable_Enable;

uint8_t Engineer_Arm_Init(void);
void Engineer_Arm_Task(const Arm_Motor_Group_t *feedback,
                       const DBUS_Typedef *dbus,
                       float dt_s);

#endif
