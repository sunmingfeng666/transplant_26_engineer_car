#ifndef H7_FRAMEWORK_ENGINEER_PICTURE_CTRL_H
#define H7_FRAMEWORK_ENGINEER_PICTURE_CTRL_H

#include <stdint.h>
#include "Robot_Config.h"

typedef enum {
    ENGINEER_PICTURE_WAIT_FEEDBACK = 0,
    ENGINEER_PICTURE_TRACKING,
    ENGINEER_PICTURE_HOMING,
    ENGINEER_PICTURE_STOPPED,
    ENGINEER_PICTURE_FAULT
} Engineer_Picture_State_e;

typedef struct {
    Engineer_Picture_State_e state;
    int32_t lift_position;
    int32_t transverse_position;
    uint8_t lift_bottom;
    uint8_t transverse_zero;
    uint8_t lift_done;
    uint8_t transverse_done;
    uint8_t homing_active;
    uint8_t homing_done;
    uint8_t fault;
} Engineer_Picture_Status_t;

uint8_t Engineer_Picture_Init(void);
void Engineer_Picture_Task(const Picture_Motor_Group_t *p_motor);
Engineer_Picture_Status_t Engineer_Picture_Get_Status(void);

#endif
