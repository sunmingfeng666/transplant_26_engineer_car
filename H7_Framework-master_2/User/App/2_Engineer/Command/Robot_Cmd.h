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
