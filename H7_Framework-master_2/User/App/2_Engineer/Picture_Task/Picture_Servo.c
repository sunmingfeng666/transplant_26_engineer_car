#include "Picture_Servo.h"
#include "Robot_Config.h"

static uint8_t servo_is_started = 0U;

uint16_t Picture_Servo_Clamp_Us(int32_t pulse_us)
{
    if (pulse_us < (int32_t)PICTURE_SERVO_MIN_US) return PICTURE_SERVO_MIN_US;
    if (pulse_us > (int32_t)PICTURE_SERVO_MAX_US) return PICTURE_SERVO_MAX_US;
    return (uint16_t)pulse_us;
}

void Picture_Servo_Set(uint16_t yaw_us, uint16_t pitch_us)
{
    BSP_PWM_Set_Compare(&picture_yaw_pwm, Picture_Servo_Clamp_Us(yaw_us));
    BSP_PWM_Set_Compare(&picture_pitch_pwm, Picture_Servo_Clamp_Us(pitch_us));
}

void Picture_Servo_Init(void)
{
    if (!servo_is_started) {
        BSP_PWM_Start(&picture_yaw_pwm);
        BSP_PWM_Start(&picture_pitch_pwm);
        servo_is_started = 1U;
    }

    Picture_Servo_Set(PICTURE_SERVO_YAW_DEFAULT_US, PICTURE_SERVO_PITCH_DEFAULT_US);
}
