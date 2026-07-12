//
// Created by CaoKangqi on 2026/6/23.
// 模块功能：总控中心逻辑（纯净版）- 指令集与接口
//
#ifndef H7_FRAMEWORK_ROBOT_CMD_H
#define H7_FRAMEWORK_ROBOT_CMD_H

#include <stdint.h>
#include <stdbool.h>

// 底盘控制指令
typedef enum {
    CHASSIS_CMD_SAFE = 0,    // 安全锁死，无输出
    CHASSIS_CMD_FOLLOW,      // 底盘跟随云台
    CHASSIS_CMD_FREE,        // 底盘与云台分离
    CHASSIS_CMD_SPIN         // 小陀螺模式
} Chassis_Mode_e;

typedef struct {
    Chassis_Mode_e mode;
    float target_vx;         // 目标 X 轴平移速度 (m/s)
    float target_vy;         // 目标 Y 轴平移速度 (m/s)
    float target_vw;         // 目标自旋角速度 (rad/s)
    float offset_angle;      // 云台与底盘的相对夹角
} Chassis_Cmd_t;

void Robot_Cmd_Init(void);
void Robot_Cmd_Update(void);

#endif //H7_FRAMEWORK_ROBOT_CMD_H
