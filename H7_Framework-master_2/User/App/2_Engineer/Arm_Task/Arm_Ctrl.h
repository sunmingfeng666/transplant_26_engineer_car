#ifndef H7_FRAMEWORK_ENGINEER_ARM_CTRL_H
#define H7_FRAMEWORK_ENGINEER_ARM_CTRL_H

#include <stdint.h>

#include "Arm_JointController.h"
#include "DBUS.h"
#include "VT13.h"
#include "Robot_Config.h"

#define ARM_BUILD_MODE_NORMAL   0U
#define ARM_BUILD_MODE_DISABLED 1U
#define ARM_BUILD_MODE_GRAVITY_ONLY 2U
#define ARM_BUILD_MODE_CALIBRATION 3U

#ifndef ARM_CONTROL_BUILD_MODE
#define ARM_CONTROL_BUILD_MODE ARM_BUILD_MODE_NORMAL
#endif

#if (ARM_CONTROL_BUILD_MODE != ARM_BUILD_MODE_NORMAL) && \
    (ARM_CONTROL_BUILD_MODE != ARM_BUILD_MODE_DISABLED) && \
    (ARM_CONTROL_BUILD_MODE != ARM_BUILD_MODE_GRAVITY_ONLY) && \
    (ARM_CONTROL_BUILD_MODE != ARM_BUILD_MODE_CALIBRATION)
#error "ARM_CONTROL_BUILD_MODE must be NORMAL, DISABLED, GRAVITY_ONLY or CALIBRATION"
#endif

uint8_t Engineer_Arm_Init(void);
void Engineer_Arm_Task(const Arm_Motor_Group_t *feedback,
                       const DBUS_Typedef *dbus,
                       const VT13_Typedef *vt13,
                       float dt_s);

#endif
