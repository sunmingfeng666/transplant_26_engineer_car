#ifndef H7_FRAMEWORK_PICTURE_SERVO_H
#define H7_FRAMEWORK_PICTURE_SERVO_H

#include <stdint.h>

#define PICTURE_SERVO_MIN_US 500U
#define PICTURE_SERVO_MAX_US 2500U
#define PICTURE_SERVO_YAW_DEFAULT_US 750U
#define PICTURE_SERVO_PITCH_DEFAULT_US 2300U

void Picture_Servo_Init(void);
void Picture_Servo_Set(uint16_t yaw_us, uint16_t pitch_us);
uint16_t Picture_Servo_Clamp_Us(int32_t pulse_us);

#endif
