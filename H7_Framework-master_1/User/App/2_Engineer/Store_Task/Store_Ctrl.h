#ifndef H7_FRAMEWORK_ENGINEER_STORE_CTRL_H
#define H7_FRAMEWORK_ENGINEER_STORE_CTRL_H

#include <stdint.h>
#include "Robot_Config.h"

typedef enum {
    ENGINEER_STORE_WAIT_FEEDBACK = 0,
    ENGINEER_STORE_ENABLING,
    ENGINEER_STORE_HOLDING,
    ENGINEER_STORE_MOVING,
    ENGINEER_STORE_STOPPED,
    ENGINEER_STORE_FAULT
} Engineer_Store_State_e;

typedef struct {
    Engineer_Store_State_e state;
    uint8_t online;
    uint8_t done;
    uint8_t fault;
    uint8_t slot;
    float position;
    float target;
} Engineer_Store_Status_t;

uint8_t Engineer_Store_Init(void);
void Engineer_Store_Task(const Store_Motor_Group_t *motors);
Engineer_Store_Status_t Engineer_Store_Get_Status(void);

#endif
