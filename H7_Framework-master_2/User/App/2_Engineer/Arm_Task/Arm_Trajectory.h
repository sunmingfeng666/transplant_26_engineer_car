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
 * 仅移植 4 条纯关节轨迹（不含存矿/取矿，那些依赖升降/横移/舵机，
 * 本板无对应机构）：展开、收回、自保护1、自保护2。
 */

typedef enum {
    ARM_TRAJ_UNFOLD = 0,  // 展开：正常 -> 取矿姿态
    ARM_TRAJ_FOLD,        // 收回：取矿姿态 -> 正常
    ARM_TRAJ_SELF_1,      // 自保护姿态 1
    ARM_TRAJ_SELF_2,      // 自保护姿态 2
    ARM_TRAJ_COUNT
} Arm_Traj_e;

/*
 * 求某条轨迹在 current_t(s) 时刻、关节 joint_idx0(0..5) 的目标角(rad)。
 * current_t 会被内部钳到 [0, total_time]。无效入参返回 0。
 */
float Arm_Traj_GetJoint(Arm_Traj_e traj, uint8_t joint_idx0, float current_t);

/* 返回该轨迹的总时长(s，压缩后)。无效 traj 返回 0。 */
float Arm_Traj_TotalTime(Arm_Traj_e traj);

#endif /* H7_FRAMEWORK_ARM_TRAJECTORY_H */
