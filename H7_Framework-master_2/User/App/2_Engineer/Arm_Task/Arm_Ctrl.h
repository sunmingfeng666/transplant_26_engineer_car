#ifndef H7_FRAMEWORK_ENGINEER_ARM_CTRL_H
#define H7_FRAMEWORK_ENGINEER_ARM_CTRL_H

#include <stdint.h>

#include "Arm_JointController.h"
#include "DBUS.h"
#include "Robot_Config.h"

uint8_t Engineer_Arm_Init(void);
void Engineer_Arm_Task(const Arm_Motor_Group_t *feedback,
                       const DBUS_Typedef *dbus,
                       float dt_s);

#endif
