//
// 机械臂第一阶段调通控制。
// J1 用达妙 DM4310 走位置速度模式，J2~J6 用 DJI 电机走软件位置环(PD 出电流)。
// 当前仅实现遥控器手动点动，尚无一键/轨迹功能。
//

#include "Arm_Ctrl.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "All_define.h"
#include "DM_Motor.h"
#include "DJI_Motor.h"
#include "Horizon_MATH.h"
#include "Robot_Config.h"
#include "fdcan.h"

#define ARM_JOINT_NUM              6U       // 关节数量
#define ARM_J1_DM_ID               0x01U    // J1 达妙电机的 CAN ID

#define ARM_DBUS_DEADBAND          20       // DBUS 摇杆死区，小于此值视为不动
#define ARM_VT13_DEADBAND          30       // VT13 摇杆死区

// 调试模式开关：置 1 时机械臂只接受 DBUS 手动点动，完全忽略 VT13；正式固件保持 0。
#define ARM_DBUS_DEBUG             0

// 点动步进：每周期摇杆量 × 步进 = 目标增量
#define ARM_J1_STEP_RAD            0.0000008f  // J1 每单位摇杆量对应的目标角增量(弧度)
#define ARM_DJI_STEP_ENCODER       0.0009f     // J2~J5 每单位摇杆量对应的编码器目标增量
#define ARM_J6_STEP_ENCODER        0.45f       // J6 按键点动的固定编码器步进

// 目标限幅范围
#define ARM_J1_MIN_RAD            -2.8f         // J1 目标角下限(弧度)
#define ARM_J1_MAX_RAD             2.8f         // J1 目标角上限(弧度)
#define ARM_DJI_MIN_ENCODER       -65536.0f     // J2~J6 目标编码器下限
#define ARM_DJI_MAX_ENCODER        65536.0f     // J2~J6 目标编码器上限

#define ARM_J1_MAX_SPEED           0.6f         // J1 位置速度模式的速度上限
#define ARM_DJI_MAX_CURRENT        1500.0f      // J2~J6 输出电流限幅

typedef struct {
    float kp;
    float kd;
} Arm_Dji_Gain_t;

static bool arm_initialized = false;              // 是否已完成初始化
static uint8_t target_seed_mask = 0U;             // 各关节"目标已播种"标志位(bit0~bit5 对应 J1~J6)
static float last_dji_error[ARM_JOINT_NUM] = {0.0f};  // 上周期误差，用于 PD 的微分项

// J2~J6 的位置环 PD 增益(kp/kd)；J1 走达妙自带位置环，此处留 0 不用
static const Arm_Dji_Gain_t dji_gains[ARM_JOINT_NUM] = {
    {0.0f, 0.0f},
    {0.35f, 2.0f},
    {0.35f, 2.0f},
    {0.35f, 2.0f},
    {0.28f, 1.5f},
    {0.28f, 1.5f},
};

// DBUS 摇杆死区处理：死区内归零，防止零点漂移导致缓慢乱动
static int16_t dbus_axis(int16_t value)
{
    return (MATH_ABS_int16_t(value) > ARM_DBUS_DEADBAND) ? value : 0;
}

// VT13 摇杆死区处理
static int16_t vt13_axis(int16_t value)
{
    return (MATH_ABS_int16_t(value) > ARM_VT13_DEADBAND) ? value : 0;
}

static float limit_float(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static int16_t limit_current(float value)
{
    value = limit_float(value, -ARM_DJI_MAX_CURRENT, ARM_DJI_MAX_CURRENT);
    return (int16_t)value;
}

// 查询指定关节是否在线(CAN 是否有反馈)
static bool joint_is_online(uint8_t joint)
{
    switch (joint) {
        case 0: return engineer_custom_motors.J1_DM4310.offline.is_online;
        case 1: return engineer_custom_motors.J2_DJI2006.offline.is_online;
        case 2: return engineer_custom_motors.J3_DJI3508.offline.is_online;
        case 3: return engineer_custom_motors.J4_DJI3508.offline.is_online;
        case 4: return engineer_custom_motors.J5_DJI2006.offline.is_online;
        case 5: return engineer_custom_motors.J6_DJI2006.offline.is_online;
        default: return false;
    }
}

// 读取 J2~J6 的无限累加编码器值；J1(case 0)不走此路，单独用 .pos，故返回 0
static int32_t joint_encoder(uint8_t joint)
{
    switch (joint) {
        case 1: return engineer_custom_motors.J2_DJI2006.Angle_Infinite;
        case 2: return engineer_custom_motors.J3_DJI3508.Angle_Infinite;
        case 3: return engineer_custom_motors.J4_DJI3508.Angle_Infinite;
        case 4: return engineer_custom_motors.J5_DJI2006.Angle_Infinite;
        case 5: return engineer_custom_motors.J6_DJI2006.Angle_Infinite;
        default: return 0;
    }
}

// 给单个关节"播种"目标：每个关节首次上线时把目标对齐到当前位置，误差归零、原地待命。
// 靠 target_seed_mask 保证每个关节只播一次；已播种或未上线则直接返回。
static void seed_joint_target(uint8_t joint)
{
    uint8_t mask = (uint8_t)(1U << joint);
    if ((target_seed_mask & mask) != 0U || !joint_is_online(joint)) {
        return;  // 已播过种 或 关节还没上线，跳过
    }

    // 首次收到反馈时用当前位置作为目标，避免上电后目标从 0 突跳。
    if (joint == 0U) {
        engineer_custom_motors.target_pos[0] = engineer_custom_motors.J1_DM4310.pos;
    } else {
        engineer_custom_motors.target_pos[joint] = (float)joint_encoder(joint);
    }
    target_seed_mask |= mask;  // 置位：标记该关节已播种
}

// 逐个关节尝试播种；每周期调用，谁刚上线就补播谁，直到全部播完
static void seed_all_online_targets(void)
{
    for (uint8_t i = 0; i < ARM_JOINT_NUM; i++) {
        seed_joint_target(i);
    }
}

// 把所有目标强制拉回当前反馈值(急停/停止时用)：清播种标志后重新播种，并清空误差和输出
static void reset_targets_to_feedback(void)
{
    target_seed_mask = 0U;
    seed_all_online_targets();
    memset(last_dji_error, 0, sizeof(last_dji_error));
    memset(engineer_custom_motors.output_current, 0, sizeof(engineer_custom_motors.output_current));
}

// 刷新在线掩码；关节一旦掉线，清掉它的播种标志、误差和输出，使其重连时重新播种防突跳
static void update_online_mask(void)
{
    uint8_t mask = 0U;
    for (uint8_t i = 0; i < ARM_JOINT_NUM; i++) {
        if (joint_is_online(i)) {
            mask |= (uint8_t)(1U << i);
        } else {
            target_seed_mask &= (uint8_t)~(1U << i);
            last_dji_error[i] = 0.0f;
            engineer_custom_motors.output_current[i] = 0;
        }
    }
    engineer_custom_motors.online_mask = mask;
}

// 给关节目标叠加增量；未播种或掉线的关节不响应，防止在无效基准上累加
static void add_joint_target(uint8_t joint, float delta)
{
    uint8_t mask = (uint8_t)(1U << joint);
    if ((target_seed_mask & mask) == 0U || !joint_is_online(joint)) {
        return;
    }
    engineer_custom_motors.target_pos[joint] += delta;
}

// 读取 DBUS 摇杆/开关，转换成各关节目标增量(手动点动)
static void apply_dbus_manual_input(const DBUS_Typedef *dbus)
{
    add_joint_target(0, (float)dbus_axis(dbus->Remote.CH3) * ARM_J1_STEP_RAD);
    add_joint_target(1, (float)dbus_axis(dbus->Remote.CH2) * ARM_DJI_STEP_ENCODER);
    add_joint_target(2, (float)dbus_axis(dbus->Remote.CH1) * ARM_DJI_STEP_ENCODER);
    add_joint_target(3, (float)dbus_axis(dbus->Remote.CH0) * ARM_DJI_STEP_ENCODER);
    add_joint_target(4, (float)dbus_axis(dbus->Remote.Dial) * ARM_DJI_STEP_ENCODER);

    // DBUS 没有额外按键，先用 S1 上/下档给 J6 做低速点动。
    if (dbus->Remote.S1 == 1U) {
        add_joint_target(5, ARM_J6_STEP_ENCODER);
    } else if (dbus->Remote.S1 == 2U) {
        add_joint_target(5, -ARM_J6_STEP_ENCODER);
    }
}

// 读取 VT13 摇杆/按键，转换成各关节目标增量；调试模式(ARM_DBUS_DEBUG=1)下不调用，故加 unused 防警告
__attribute__((unused)) static void apply_vt13_manual_input(const VT13_Typedef *vt13)
{
    add_joint_target(0, (float)vt13_axis(vt13->Remote.Channel[3]) * ARM_J1_STEP_RAD);
    add_joint_target(1, (float)vt13_axis(vt13->Remote.Channel[2]) * ARM_DJI_STEP_ENCODER);
    add_joint_target(2, (float)vt13_axis(vt13->Remote.Channel[1]) * ARM_DJI_STEP_ENCODER);
    add_joint_target(3, (float)vt13_axis(vt13->Remote.Channel[0]) * ARM_DJI_STEP_ENCODER);
    add_joint_target(4, (float)vt13_axis(vt13->Remote.wheel) * ARM_DJI_STEP_ENCODER);

    // VT13 用左右自定义键点动 J6，扳机作为备用正向点动。
    if (vt13->Remote.fn_2 || vt13->Remote.trigger) {
        add_joint_target(5, ARM_J6_STEP_ENCODER);
    }
    if (vt13->Remote.fn_1) {
        add_joint_target(5, -ARM_J6_STEP_ENCODER);
    }
}

// 对所有关节目标限幅：J1 按弧度、J2~J6 按编码器范围，防止超出机械行程
static void limit_targets(void)
{
    engineer_custom_motors.target_pos[0] = limit_float(engineer_custom_motors.target_pos[0], ARM_J1_MIN_RAD, ARM_J1_MAX_RAD);
    for (uint8_t i = 1; i < ARM_JOINT_NUM; i++) {
        engineer_custom_motors.target_pos[i] = limit_float(engineer_custom_motors.target_pos[i],
                                                        ARM_DJI_MIN_ENCODER,
                                                        ARM_DJI_MAX_ENCODER);
    }
}

// 单个 DJI 关节的位置环 PD：输出 = kp*误差 + kd*误差变化量(固定 1ms 周期，dt 折进 kd)
static int16_t calc_dji_output(uint8_t joint)
{
    float measure = (float)joint_encoder(joint);
    float error = engineer_custom_motors.target_pos[joint] - measure;
    float output = dji_gains[joint].kp * error + dji_gains[joint].kd * (error - last_dji_error[joint]);
    last_dji_error[joint] = error;
    return limit_current(output);
}

// 下发所有关节输出：J1 发位置速度指令，J2~J6 算电流后分两帧经两路 CAN 发送
static void send_outputs(void)
{
    // J1：达妙位置速度模式，直接下发目标角和限速
    float j1_target = engineer_custom_motors.target_pos[0];
    engineer_custom_motors.target_vel[0] = ARM_J1_MAX_SPEED;
    Pos_Speed_Ctrl(&hfdcan2, ARM_J1_DM_ID, j1_target, ARM_J1_MAX_SPEED);

    // J2~J6：在线才算电流，掉线输出清 0
    for (uint8_t i = 1; i < ARM_JOINT_NUM; i++) {
        if (joint_is_online(i)) {
            engineer_custom_motors.output_current[i] = calc_dji_output(i);
        } else {
            engineer_custom_motors.output_current[i] = 0;
        }
    }

    // CAN1 的 0x200 帧承载 J2/J3/J4(首字节留给电调 ID1，故填 0)
    DJI_Motor_Send(&hfdcan1, 0x200,
                   0,
                   engineer_custom_motors.output_current[1],
                   engineer_custom_motors.output_current[2],
                   engineer_custom_motors.output_current[3]);
    // CAN2 的 0x1FF 帧承载 J5/J6
    DJI_Motor_Send(&hfdcan2, 0x1FF,
                   engineer_custom_motors.output_current[4],
                   engineer_custom_motors.output_current[5],
                   0,
                   0);
}

// 机械臂初始化：清空目标与输出、复位播种标志，并把 J1 达妙电机切到位置速度模式
void Arm_Ctrl_Init(void)
{
    memset(engineer_custom_motors.target_pos, 0, sizeof(engineer_custom_motors.target_pos));
    memset(engineer_custom_motors.target_vel, 0, sizeof(engineer_custom_motors.target_vel));
    memset(engineer_custom_motors.output_current, 0, sizeof(engineer_custom_motors.output_current));
    target_seed_mask = 0U;
    arm_initialized = true;

    // J1 使用达妙位置速度模式；未收到反馈前不发送位置目标。
    Motor_Mode(&hfdcan2, ARM_J1_DM_ID, POS_MODE, DM_CMD_MOTOR_MODE);
}

// 机械臂主控周期函数：更新在线状态→播种→读遥控输入叠加目标→限幅→下发
void Arm_Ctrl_Update(const DBUS_Typedef *dbus)
{
    if (!arm_initialized) {
        Arm_Ctrl_Init();
    }
    if (dbus == NULL ) {
        Arm_Ctrl_Stop();  // 两个遥控源都没有，直接停
        return;
    }

    update_online_mask();
    seed_all_online_targets();

#if ARM_DBUS_DEBUG
    // 调试模式：只认 DBUS
    // DBUS 缺失/掉线/S2 打到急停位 → 停止
    if (dbus == NULL || !dbus->offline.is_online || dbus->Remote.S2 == 2U) {
        Arm_Ctrl_Stop();
        return;
    }

    apply_dbus_manual_input(dbus);
#else
    // 正式模式：DBUS 的 S2 急停位任一触发都停止
    if ((dbus != NULL && dbus->offline.is_online && dbus->Remote.S2 == 2U)) {
        Arm_Ctrl_Stop();
        return;
    }

    // VT13 在线优先用 VT13，否则回退到 DBUS，都不可用则停止
    if (dbus != NULL && dbus->offline.is_online) {
        apply_dbus_manual_input(dbus);
    } else {
        Arm_Ctrl_Stop();
        return;
    }
#endif

    limit_targets();
    send_outputs();
}

void Arm_Ctrl_Stop(void)
{
    if (!arm_initialized) {
        return;
    }

    update_online_mask();
    reset_targets_to_feedback();

    if (engineer_custom_motors.J1_DM4310.offline.is_online) {
        Pos_Speed_Ctrl(&hfdcan2, ARM_J1_DM_ID, engineer_custom_motors.target_pos[0], 0.0f);
    }
    DJI_Motor_Send(&hfdcan1, 0x200, 0, 0, 0, 0);
    DJI_Motor_Send(&hfdcan2, 0x1FF, 0, 0, 0, 0);
}
