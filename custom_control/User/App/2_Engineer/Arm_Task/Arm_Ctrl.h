//
// Mechanical arm first-stage bring-up control.
//

#ifndef H7_FRAMEWORK_ARM_CTRL_H
#define H7_FRAMEWORK_ARM_CTRL_H

#include "DBUS.h"
#include "VT13.h"

void Arm_Ctrl_Init(void);
void Arm_Ctrl_Update(const DBUS_Typedef *dbus);
void Arm_Ctrl_Stop(void);

#endif //H7_FRAMEWORK_ARM_CTRL_H
