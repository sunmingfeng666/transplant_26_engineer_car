#ifndef H7_FRAMEWORK_ARM_TRAJECTORY_H
#define H7_FRAMEWORK_ARM_TRAJECTORY_H

#include <stdint.h>

/*
 * 机械臂一键轨迹数据层（移植自旧臂主控 One_Click_Task.c）
 *
 * 纯数据 + 三次多项式插值，框架无关。轨迹在 MATLAB 离线规划，
 * 导出为分段三次多项式系数 q_coeffs[关节][分段][a,b,c,d]。
 * 关节定义与旧框架一致(J1..J6)，系数表原样复用，无需重新标定。
 *
 * 共包含 8 条轨迹：展开、收回、自保护 1/2，以及存矿、取矿各两段。
 * 机械臂轨迹只负责关节目标，升降、横移、存矿电机和舵机由一键动作状态机协调。
 */

typedef enum {
    ARM_TRAJ_UNFOLD = 0,  // 展开：正常 -> 取矿姿态
    ARM_TRAJ_FOLD,        // 收回：取矿姿态 -> 正常
    ARM_TRAJ_SELF_1,      // 自保护姿态 1
    ARM_TRAJ_SELF_2,      // 自保护姿态 2
    ARM_TRAJ_STORE_1,     // 存矿第一段
    ARM_TRAJ_STORE_2,     // 存矿第二段
    ARM_TRAJ_TAKE_1,      // 取矿第一段
    ARM_TRAJ_TAKE_2,      // 取矿第二段
    ARM_TRAJ_TO_ZERO,     // 实车收起HOME -> 机械零点
    ARM_TRAJ_PREGRASP_1,  // 机械零点 -> 1号预抓取位
    ARM_TRAJ_PREGRASP_2,
    ARM_TRAJ_PREGRASP_3,
    ARM_TRAJ_PREGRASP_4,
    ARM_TRAJ_PREGRASP_5,
    ARM_TRAJ_PREGRASP_6,
    ARM_TRAJ_COUNT
} Arm_Traj_e;

/*
 * 求某条轨迹在 current_t(s) 时刻、关节 joint_idx0(0..5) 的目标角(rad)。
 * current_t 会被内部钳到 [0, total_time]。无效入参返回 0。
 */
float Arm_Traj_GetJoint(Arm_Traj_e traj, uint8_t joint_idx0, float current_t);

/* 返回该轨迹的总时长(s，压缩后)。无效 traj 返回 0。 */
float Arm_Traj_TotalTime(Arm_Traj_e traj);

/* 轨迹数据是否可执行。预抓取标定未完成时返回0，防止误上车。 */
uint8_t Arm_Traj_IsAvailable(Arm_Traj_e traj);

#endif /* H7_FRAMEWORK_ARM_TRAJECTORY_H */
