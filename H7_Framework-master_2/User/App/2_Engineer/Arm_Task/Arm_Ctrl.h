#ifndef H7_FRAMEWORK_ENGINEER_ARM_CTRL_H
#define H7_FRAMEWORK_ENGINEER_ARM_CTRL_H

#include <stdint.h>

#include "Arm_JointController.h"
#include "DBUS.h"
#include "VT13.h"
#include "Robot_Config.h"

/*
 * 机械臂编译模式：调试时修改 ARM_CONTROL_BUILD_MODE，然后重新编译并烧录。
 * NORMAL 保持正常控制；DISABLED 固定让 J1~J6 和末端夹爪进入复位失能状态；
 * GRAVITY_ONLY 仅让已有模型的 J2/J4/J5 运行纯重力补偿，其余关节保持当前位置。
 * 这里不提供运行时切换，防止机械臂运动过程中误改模式。
 */
#define ARM_BUILD_MODE_NORMAL   0U
#define ARM_BUILD_MODE_DISABLED 1U
#define ARM_BUILD_MODE_GRAVITY_ONLY 2U

#ifndef ARM_CONTROL_BUILD_MODE
#define ARM_CONTROL_BUILD_MODE ARM_BUILD_MODE_NORMAL
#endif

#if (ARM_CONTROL_BUILD_MODE != ARM_BUILD_MODE_NORMAL) && \
    (ARM_CONTROL_BUILD_MODE != ARM_BUILD_MODE_DISABLED) && \
    (ARM_CONTROL_BUILD_MODE != ARM_BUILD_MODE_GRAVITY_ONLY)
#error "ARM_CONTROL_BUILD_MODE must be NORMAL, DISABLED or GRAVITY_ONLY"
#endif

uint8_t Engineer_Arm_Init(void);
void Engineer_Arm_Task(const Arm_Motor_Group_t *feedback,
                       const DBUS_Typedef *dbus,
                       const VT13_Typedef *vt13,
                       float dt_s);

#endif
