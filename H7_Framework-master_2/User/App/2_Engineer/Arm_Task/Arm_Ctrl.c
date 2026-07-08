//
// 工程机械臂控制实现（第一阶段：基础遥操作）
// 移植自旧臂主控 DM_H7_Master（2）的 Move_Task.c。
// 关键适配：
//   1) 新框架 Pos_Speed_Ctrl/MIT_Ctrl 为传值接口（旧为传结构体指针）；
//   2) FDCAN_Send_Msg 底层非阻塞，去掉旧代码每帧间的 osDelay(2)；
//   3) 在线判断改用 Offline_Detector 的 offline.is_online。
//
#include "Arm_Ctrl.h"
#include <math.h>
#include "BSP_FDCAN.h"
#include "DM_Motor.h"
#include "fdcan.h"

// --- 关节限幅（弧度），沿用旧臂 Move_Task.c 的 Boundary 常量。⚠️ 换机械结构需重标 ---
#define ARM_J2_MAX    3.67875862f
#define ARM_J2_MIN   -0.532349f
#define ARM_J4_MAX    1.848f
#define ARM_J4_MIN   -1.8297427f
#define ARM_J5_MAX    1.67410f
#define ARM_J5_MIN   -1.76146317f

// --- 位置速度模式的速度上限，沿用旧臂 All_Init.c 初值 ---
#define ARM_JOINT_SPEED   2.0f

// --- 末端夹爪 MIT 力矩，沿用旧臂 Move_Task.c：闭合 +1.0 / 张开 -1.2 ---
#define TERMINAL_TORQUE_CLOSE   1.0f
#define TERMINAL_TORQUE_OPEN   -1.2f

// --- 遥控增量步长，沿用旧臂 RUI_DBUS.c 手感（每通道 ±0.00005 rad/拍）---
#define ARM_DBUS_STEP   0.00005f

// 上电缓抬标志：0=尚未缓抬到位，1=已就绪可正常控制（旧 power_on_arm 逻辑）
static uint8_t s_power_on_arm = 0;

// 6 个关节的目标位置（弧度）+ 末端夹爪开合（1=闭合,0=张开）
static float   s_joint_target[6] = {0};
static uint8_t s_clamp_close = 1;

// 单关节上下限截断
static float Clamp_f(float v, float min, float max)
{
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

/**
 * @brief 机械臂初始化：设置各关节位置速度模式的速度上限与末端初态。
 * @note  电机的使能/清错在 Task 里按在线状态动态处理，这里只初始化软件目标。
 */
uint8_t Engineer_Arm_Init(void)
{
    s_power_on_arm = 0;
    s_clamp_close  = 1;               // 上电默认夹爪闭合，与旧臂 All_Init 一致
    for (int i = 0; i < 6; i++) {
        s_joint_target[i] = 0.0f;
    }
    return 0;
}

/**
 * @brief 遥控输入 → 关节目标增量映射（沿用旧臂 RUI_DBUS.c / VT13.c 手感）。
 * @note  6关节 > 4摇杆通道，故用档位开关分两组：
 *        DBUS S1=上(1)：CH0→J1(反向) CH1→J2 CH2→J3 CH3→J4
 *        DBUS S1=下(2)：CH0→J5      CH1→J6
 *        DBUS S2=上→夹爪闭合，S2=下→夹爪张开
 *        VT13 摇杆/拨轮同时叠加增量，pause 键切换夹爪。
 */
static void Arm_Input_Update(const DBUS_Typedef *dbus, const VT13_Typedef *vt13)
{
    // ---- DBUS：按 S1 档位选择关节组，增量叠加 ----
    if (dbus != NULL && dbus->offline.is_online) {
        if (dbus->Remote.S1 == 1) {            // 上档：J1~J4
            s_joint_target[0] += (float)dbus->Remote.CH0 * -ARM_DBUS_STEP;
            s_joint_target[1] += (float)dbus->Remote.CH1 *  ARM_DBUS_STEP;
            s_joint_target[2] += (float)dbus->Remote.CH2 *  ARM_DBUS_STEP;
            s_joint_target[3] += (float)dbus->Remote.CH3 *  ARM_DBUS_STEP;
        } else if (dbus->Remote.S1 == 2) {     // 下档：J5~J6
            s_joint_target[4] += (float)dbus->Remote.CH0 * ARM_DBUS_STEP;
            s_joint_target[5] += (float)dbus->Remote.CH1 * ARM_DBUS_STEP;
        }
        if (dbus->Remote.S2 == 1)      s_clamp_close = 1; // 上：闭合
        else if (dbus->Remote.S2 == 2) s_clamp_close = 0; // 下：张开
    }

    // ---- VT13 图传遥控：摇杆/拨轮增量叠加，pause 切夹爪 ----
    if (vt13 != NULL && vt13->offline.is_online) {
        s_joint_target[0] += (float)vt13->Remote.Channel[0] * -ARM_DBUS_STEP;
        s_joint_target[1] += (float)vt13->Remote.Channel[1] *  ARM_DBUS_STEP;
        s_joint_target[2] += (float)vt13->Remote.Channel[3] *  ARM_DBUS_STEP;
        s_joint_target[3] += (float)vt13->Remote.Channel[2] *  ARM_DBUS_STEP;
        s_joint_target[5] += (float)vt13->Remote.wheel      *  ARM_DBUS_STEP;
        s_joint_target[4] += (float)vt13->Remote.fn_1 * 0.005f - (float)vt13->Remote.fn_2 * 0.005f;
        if (vt13->Remote.pause == 1) s_clamp_close = 0; else s_clamp_close = 1;
    }

    // ---- 关节限幅（沿用旧 Motion_limitation 的 J2/J4/J5 边界）----
    s_joint_target[1] = Clamp_f(s_joint_target[1], ARM_J2_MIN, ARM_J2_MAX);
    s_joint_target[3] = Clamp_f(s_joint_target[3], ARM_J4_MIN, ARM_J4_MAX);
    s_joint_target[4] = Clamp_f(s_joint_target[4], ARM_J5_MIN, ARM_J5_MAX);
}

// 单关节电机：若离线则清错并重新使能（旧代码在 ONLINE_JUDGE_TIME==0 时做同样的事）。
// hcan 发送句柄，id 为电机 base id（0x01~0x07），mode 为 POS_MODE/MIT_MODE。
// 返回 1 表示当前在线，0 表示离线（本拍已尝试重连）。
static uint8_t Arm_Motor_Keepalive(FDCAN_HandleTypeDef *hcan, uint16_t id,
                                   uint16_t mode, const DM_MOTOR_DATA_Typedef *m)
{
    if (!m->offline.is_online) {
        Motor_Mode(hcan, id, mode, DM_CMD_CLEAR_ERROR);
        Motor_Mode(hcan, id, mode, DM_CMD_MOTOR_MODE);
        return 0;
    }
    return 1;
}

/**
 * @brief 机械臂主控制任务，由 Motor_Task 以 1kHz 调用。
 * @param a_motor 机械臂电机反馈组（只读）
 * @param dbus    DBUS 遥控快照（只读，可为 NULL）
 * @param vt13    VT13 图传遥控快照（只读，可为 NULL）
 * @note  无 osDelay：FDCAN 底层非阻塞，所有发送在 1ms 内完成。
 */
void Engineer_Arm_Task(const Arm_Motor_Group_t *a_motor, const DBUS_Typedef *dbus, const VT13_Typedef *vt13)
{
    if (a_motor == NULL) return;

    // ---- 上电缓抬：等 J2 反馈位置回到接近零位再放行正常控制，防止上电瞬间冲击 ----
    if (s_power_on_arm == 0) {

        if (fabsf(a_motor->J2_8009.pos) < 0.1f) {
            s_power_on_arm = 1;
            // 目标初始化为当前反馈位置，避免放行瞬间跳变
            s_joint_target[0] = a_motor->J1_8009.pos;
            s_joint_target[1] = a_motor->J2_8009.pos;
            s_joint_target[2] = a_motor->J3_4340.pos;
            s_joint_target[3] = a_motor->J4_4340.pos;
            s_joint_target[4] = a_motor->J5_4310.pos;
            s_joint_target[5] = a_motor->J6_4310.pos;
        }
        return; // 未就绪不发控制帧
    }

    // ---- 遥控输入 → 关节目标（含限幅）----
    Arm_Input_Update(dbus, vt13);

    // ---- 电机保活：离线的关节清错重使能，全部在线才允许下发控制帧 ----
    uint8_t all_online = 1;
    all_online &= Arm_Motor_Keepalive(&hfdcan2, 0x01, POS_MODE, &a_motor->J1_8009);
    all_online &= Arm_Motor_Keepalive(&hfdcan2, 0x02, POS_MODE, &a_motor->J2_8009);
    all_online &= Arm_Motor_Keepalive(&hfdcan2, 0x03, POS_MODE, &a_motor->J3_4340);
    all_online &= Arm_Motor_Keepalive(&hfdcan3, 0x04, POS_MODE, &a_motor->J4_4340);
    all_online &= Arm_Motor_Keepalive(&hfdcan3, 0x05, POS_MODE, &a_motor->J5_4310);
    all_online &= Arm_Motor_Keepalive(&hfdcan3, 0x06, POS_MODE, &a_motor->J6_4310);
    all_online &= Arm_Motor_Keepalive(&hfdcan3, 0x07, MIT_MODE, &a_motor->Terminal_3507);

    if (!all_online) return;

    // ---- 下发控制帧：J1~J6 位置速度模式，末端夹爪 MIT 力矩模式 ----
    Pos_Speed_Ctrl(&hfdcan2, 0x01, s_joint_target[0], ARM_JOINT_SPEED);
    Pos_Speed_Ctrl(&hfdcan2, 0x02, s_joint_target[1], ARM_JOINT_SPEED);
    Pos_Speed_Ctrl(&hfdcan2, 0x03, s_joint_target[2], ARM_JOINT_SPEED);
    Pos_Speed_Ctrl(&hfdcan3, 0x04, s_joint_target[3], ARM_JOINT_SPEED);
    Pos_Speed_Ctrl(&hfdcan3, 0x05, s_joint_target[4], ARM_JOINT_SPEED);
    Pos_Speed_Ctrl(&hfdcan3, 0x06, s_joint_target[5], ARM_JOINT_SPEED);

    float terminal_torque = s_clamp_close ? TERMINAL_TORQUE_CLOSE : TERMINAL_TORQUE_OPEN;
    MIT_Ctrl(&hfdcan3, 0x07, 0.0f, 0.0f, 0.0f, 0.0f, terminal_torque);
}
