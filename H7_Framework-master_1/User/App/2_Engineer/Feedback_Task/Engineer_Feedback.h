#ifndef H7_FRAMEWORK_ENGINEER_FEEDBACK_H
#define H7_FRAMEWORK_ENGINEER_FEEDBACK_H

#include "Robot_Config.h"

void Engineer_Feedback_Init(void);
void Engineer_Feedback_Task(const Chassis_Motor_Group_t *chassis,
                            const Picture_Motor_Group_t *picture,
                            const Store_Motor_Group_t *store);

#endif
