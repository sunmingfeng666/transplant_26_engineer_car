#ifndef H7_FRAMEWORK_ENGINEER_LIMIT_H
#define H7_FRAMEWORK_ENGINEER_LIMIT_H

#include <stdint.h>

void Engineer_Limit_Init(void);
void Engineer_Limit_Update(void);
uint8_t Engineer_Limit_Lift_Bottom(void);
uint8_t Engineer_Limit_Transverse_Zero(void);
uint8_t Engineer_Limit_LeadScrew_Up(void);
uint8_t Engineer_Limit_LeadScrew_Down(void);

#endif
