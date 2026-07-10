#ifndef H7_FRAMEWORK_ROBOT_CMD_H
#define H7_FRAMEWORK_ROBOT_CMD_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    CHASSIS_CMD_SAFE = 0,
    CHASSIS_CMD_FOLLOW,
    CHASSIS_CMD_FREE,
    CHASSIS_CMD_SPIN
} Chassis_Mode_e;

typedef struct {
    Chassis_Mode_e mode;
    float target_vx;
    float target_vy;
    float target_vw;
    float offset_angle;
} Chassis_Cmd_t;

typedef enum {
    GIMBAL_CMD_SAFE = 0,
    GIMBAL_CMD_MANUAL,
    GIMBAL_CMD_AUTO_AIM
} Gimbal_Mode_e;

typedef struct {
    Gimbal_Mode_e mode;
    float target_pitch;
    float target_yaw;
} Gimbal_Cmd_t;

typedef enum {
    SHOOT_CMD_SAFE = 0,
    SHOOT_CMD_READY,
    SHOOT_CMD_FIRE
} Shoot_Mode_e;

typedef struct {
    Shoot_Mode_e mode;
    float friction_rpm;
    bool trigger_single;
    bool trigger_auto;
    uint8_t bullet_speed;
} Shoot_Cmd_t;

typedef struct {
    int32_t lift;
    int32_t transverse;
    uint16_t yaw_us;
    uint16_t pitch_us;
    uint8_t enable;
} Picture_Cmd_t;

void Robot_Cmd_Init(void);
void Robot_Cmd_Update(void);

#endif
