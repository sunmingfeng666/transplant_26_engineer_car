#ifndef H7_FRAMEWORK_ENGINEER_PICTURE_CTRL_H
#define H7_FRAMEWORK_ENGINEER_PICTURE_CTRL_H

#include <stdint.h>
#include "Robot_Config.h"

uint8_t Engineer_Picture_Init(void);
void Engineer_Picture_Task(const Picture_Motor_Group_t *p_motor);

#endif
