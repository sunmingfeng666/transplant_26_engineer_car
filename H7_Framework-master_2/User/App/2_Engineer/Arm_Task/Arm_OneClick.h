#ifndef H7_FRAMEWORK_ARM_ONECLICK_H
#define H7_FRAMEWORK_ARM_ONECLICK_H

#include <stdint.h>

#include "Arm_Trajectory.h"

/*
 * 机械臂一键动作引擎（移植自旧臂主控 One_Click_Task.c，仅关节部分）
 *
 * 触发方式：写 Arm_Control_Config.oneclick_request（见下方编号），
 *           引擎执行完自动清零；写 oneclick_abort=1 可随时中止。
 *
 * 工作机理：按 DWT 时间沿轨迹把目标角写入 s_target[]，交给现有阻抗/位置
 *           控制器跟踪。不切换电机模式——沿用当前 MIT 模式，跟踪力矩由
 *           阻抗控制器 kp*(target-pos)-kd*vel 产生（需先在阻抗模式调好参数）。
 *
 * 每个动作分两段：
 *   1) 前导段：从触发瞬间的当前位置，线性插值到轨迹起点（防止突跳）。
 *   2) 回放段：沿离线轨迹从 t=0 走到 total_time。
 */

/* oneclick_request 取值（0 = 无动作/空闲）。 */
typedef enum {
    ARM_ONECLICK_NONE = 0,
    ARM_ONECLICK_UNFOLD = 1,  // 展开：正常 -> 取矿姿态
    ARM_ONECLICK_FOLD = 2,    // 收回：取矿姿态 -> 正常
    ARM_ONECLICK_SELF_1 = 3,  // 自保护姿态 1
    ARM_ONECLICK_SELF_2 = 4,  // 自保护姿态 2
} Arm_OneClick_Req_e;

/* 引擎内部阶段（供遥测观察 Arm_Control_Debug.oneclick_phase）。 */
typedef enum {
    ARM_ONECLICK_PHASE_IDLE = 0,
    ARM_ONECLICK_PHASE_LEAD_IN,   // 前导：当前位置 -> 轨迹起点
    ARM_ONECLICK_PHASE_PLAYBACK,  // 回放轨迹
} Arm_OneClick_Phase_e;

/* 复位引擎（上电初始化时调用）。 */
void Arm_OneClick_Init(void);

/*
 * 每个控制周期调用一次。若有一键动作在执行，把目标角写入 target[] 并返回 1；
 * 空闲则不动 target[] 并返回 0（调用方沿用 DBUS/MATLAB 逻辑）。
 *
 * @param position 当前实测关节角(rad)，长度 count，用于前导段起点捕获
 * @param target   目标关节角数组(rad)，被本引擎覆盖写入
 * @param count    数组长度（应为 ARM_JOINT_COUNT）
 * @return 1 = 本周期接管了 target[]，0 = 未接管
 */
uint8_t Arm_OneClick_Update(const float *position, float *target, uint8_t count);

#endif /* H7_FRAMEWORK_ARM_ONECLICK_H */
