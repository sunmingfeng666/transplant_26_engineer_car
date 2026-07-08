//
// Created by CaoKangqi on 2026/6/25.
//
#include "Catapult_Ctrl.h"
#include "Message_Center.h"
#include "System_State.h"
#include "Horizon_MATH.h"
#include "BSP_TIM.h"

#define TOTAL_SLOTS         6.0f
#define FEED_ZERO_OFFSET    1325.0f
#define COUNTS_PER_SHOT     (8192.0f / TOTAL_SLOTS)

// 静态实例化内部控制块
static Shoot_Ctrl_Block_t shoot_ctrl = {0};

// Pub/Sub 句柄与本地缓存
static Subscriber_t *sys_state_sub;
static Subscriber_t *shoot_cmd_sub;
static Subscriber_t *gimbal_cmd_sub;

static System_State_t local_sys_state;
static Shoot_Cmd_t    local_shoot_cmd = {0};
static Gimbal_Cmd_t   local_gimbal_cmd = {0};

// 私有函数声明
static uint8_t Check_Motor_Reached_Limit(const DJI_MOTOR_DATA_Typedef* motor, float target_speed, float stuck_current, uint16_t confirm_time);

/**
 * @brief 发射与云台(Yaw)控制初始化
 */
uint8_t Shoot_Control_Init(void)
{
    // 初始化状态机
    shoot_ctrl.calib_state   = CALIB_START;
    shoot_ctrl.pull_state    = PULL_STATE_NORMAL;
    shoot_ctrl.last_switch_v = GPIO_PIN_SET;

    float PID_P_FEED[3] = {1.0f, 0.0f, 0.0f};
    float PID_S_FEED[3] = {0.4f, 0.0f, 0.0f};
    float PID_P_YAW[3]  = {1.6f, 0.0f, 0.0f};
    float PID_S_YAW[3]  = {8.0f, 0.01f, 0.0f};
    float PID_P_PULL[3] = {4.0f, 0.0f, 0.0f}; // 默认使用非Reset的高刚性参数
    float PID_S_PULL[3] = {7.0f, 0.0f, 0.0f};

    uint8_t mode = Integral_Limit | ErrorHandle;

    // 初始化 PID
    PID_Init(&shoot_ctrl.PID_Feed_P, 80, 30, PID_P_FEED, 0, 0, 0, 0, 0, mode);
    PID_Init(&shoot_ctrl.PID_Feed_S, 15, 10, PID_S_FEED, 0, 0, 0, 0, 0, mode);

    PID_Init(&shoot_ctrl.PID_Yaw_P, 1000, 150, PID_P_YAW, 0, 0, 0, 0, 0, mode);
    PID_Init(&shoot_ctrl.PID_Yaw_S, 16384, 2000, PID_S_YAW, 0, 0, 0, 0, 0, mode);

    PID_Init(&shoot_ctrl.PID_Pull_P, 4000, 100, PID_P_PULL, 0, 0, 0, 0, 0, mode);
    PID_Init(&shoot_ctrl.PID_Pull_S, 16384, 2000, PID_S_PULL, 0, 0, 0, 0, 0, mode);

    // 订阅消息
    sys_state_sub  = SubRegister("system_state", sizeof(System_State_t));
    shoot_cmd_sub  = SubRegister("shoot_cmd", sizeof(Shoot_Cmd_t));
    gimbal_cmd_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Cmd_t));

    // 报告节点就绪
    System_State_Report(ID_SHOOT, STATUS_PREPARING);

    return 1;
}

/**
 * @brief 发射与云台控制主任务 (集成校准与发射逻辑)
 */
void Shoot_Control_Task(const Shoot_Motor_Group_t *s_motor,
                        const Gimbal_Motor_Group_t *g_motor)
{
    // 1. 安全检查与数据获取
    if (s_motor == NULL || g_motor == NULL) {
        System_State_Report(ID_SHOOT, STATUS_ERROR);
        return;
    }

    if (sys_state_sub)  SubGetMessage(sys_state_sub, &local_sys_state);
    if (shoot_cmd_sub)  SubGetMessage(shoot_cmd_sub, &local_shoot_cmd);
    if (gimbal_cmd_sub) SubGetMessage(gimbal_cmd_sub, &local_gimbal_cmd);

    System_State_Report(ID_SHOOT, STATUS_RUN);
    System_State_Report(ID_GIMBAL, STATUS_RUN);

    // 2. 离线保护机制
    if (!Is_Group_Online(SHOOT)) {
        System_State_Report(ID_SHOOT, STATUS_LOST);
    }
    if (!Is_Group_Online(GIMBAL)){
        System_State_Report(ID_GIMBAL, STATUS_LOST);
    }

    // 3. 安全模式处理 (一键归零)
    if (local_sys_state.global_mode == GLOBAL_SAFE_LOCK ||
        local_sys_state.global_mode == GLOBAL_STANDBY ||
        local_sys_state.global_mode == GLOBAL_MODULE_ERROR)
    {
        PID_Clear(&shoot_ctrl.PID_Feed_P);
        PID_Clear(&shoot_ctrl.PID_Feed_S);
        PID_Clear(&shoot_ctrl.PID_Yaw_P);
        PID_Clear(&shoot_ctrl.PID_Yaw_S);
        PID_Clear(&shoot_ctrl.PID_Pull_P);
        PID_Clear(&shoot_ctrl.PID_Pull_S);

        // 关闭触发器 PWM
        BSP_PWM_Set_Compare(&trigger_pwm, 1200);

        // 强行发送 0 电流
        DJI_Motor_Send(&hfdcan3, 0x200, 0, 0, 0, 0);
        DM_Motor_Send(&hfdcan3, 0x3FE, 0, 0, 0, 0);
        return;
    }

    // 4. 云台 Yaw 轴限位校准状态机
    switch (shoot_ctrl.calib_state)
    {
        case CALIB_START:
            shoot_ctrl.calib_state = CALIB_MOVING;
            break;

        case CALIB_MOVING:
            PID_Calculate(&shoot_ctrl.PID_Yaw_S, g_motor->DJI_3508_Yaw.Speed_now, 500.0f);
            DJI_Motor_Send(&hfdcan3, 0x200, 0, 0, shoot_ctrl.PID_Yaw_S.Output, 0);

            if (Check_Motor_Reached_Limit(&g_motor->DJI_3508_Yaw, 500.0f, 3000.0f, 80)) {
                shoot_ctrl.calib_state = CALIB_DONE;
            }
            return; // 校准期间不执行后续发射逻辑

        case CALIB_DONE:
            DJI_Motor_Send(&hfdcan3, 0x200, 0, 0, 0, 0); // 停转卸力
            shoot_ctrl.PID_Yaw_S.Iout = 0.0f;

            shoot_ctrl.zero_offset_angle = g_motor->DJI_3508_Yaw.Angle_Infinite;
            shoot_ctrl.mid_offset_angle = shoot_ctrl.zero_offset_angle - 72550.0f;

            shoot_ctrl.PID_Yaw_P.Ref = shoot_ctrl.mid_offset_angle;
            shoot_ctrl.calib_state = CALIB_NORMAL;
            break;

        case CALIB_NORMAL:
            break; // 正常通过，向下执行
    }

    // --- 确保 DM 电机已连接 ---
    if (s_motor->DM4310_Feed.Angle_Infinite == 0.0f && s_motor->DM4310_Feed.Speed_now == 0.0f) {
        return;
    }

    // 5. 拨盘电机 (Feed) 控制逻辑
    float current_pulse = s_motor->DM4310_Feed.Angle_Infinite;

    if (!shoot_ctrl.feed_motor.is_init) {
        shoot_ctrl.feed_motor.target_pos_cnt = (int32_t)floorf((current_pulse - FEED_ZERO_OFFSET) / COUNTS_PER_SHOT);
        shoot_ctrl.feed_motor.smooth_ref = current_pulse;
        shoot_ctrl.feed_motor.is_init = true;
    }

    float final_target = FEED_ZERO_OFFSET + ((float)shoot_ctrl.feed_motor.target_pos_cnt * COUNTS_PER_SHOT);

    if (shoot_ctrl.feed_motor.smooth_ref > final_target) {
        shoot_ctrl.feed_motor.smooth_ref -= 5.0f;
        if (shoot_ctrl.feed_motor.smooth_ref < final_target) {
            shoot_ctrl.feed_motor.smooth_ref = final_target;
        }
    } else {
        shoot_ctrl.feed_motor.smooth_ref = final_target;
    }

    PID_Calculate(&shoot_ctrl.PID_Feed_P, current_pulse, shoot_ctrl.feed_motor.smooth_ref);
    PID_Calculate(&shoot_ctrl.PID_Feed_S, s_motor->DM4310_Feed.Speed_now, shoot_ctrl.PID_Feed_P.Output);

    // 6. 云台 (Yaw) 角度跟随控制逻辑
    // 注意：这里用本地指令集代替了对 DBUS 的硬编码
    shoot_ctrl.PID_Yaw_P.Ref += local_gimbal_cmd.target_yaw; // 假设 target_yaw_inc 是增量
    shoot_ctrl.PID_Yaw_P.Ref = MATH_Limit_float(shoot_ctrl.PID_Yaw_P.Ref,
                                                shoot_ctrl.mid_offset_angle - 72550.0f,
                                                shoot_ctrl.mid_offset_angle + 72550.0f);

    PID_Calculate(&shoot_ctrl.PID_Yaw_P, g_motor->DJI_3508_Yaw.Angle_Infinite, shoot_ctrl.PID_Yaw_P.Ref);
    PID_Calculate(&shoot_ctrl.PID_Yaw_S, g_motor->DJI_3508_Yaw.Speed_now, shoot_ctrl.PID_Yaw_P.Output);

    // 7. 发射触发器 (Pull) 状态机
    GPIO_PinState current_switch_v = HAL_GPIO_ReadPin(Switch_GPIO_Port, Switch_Pin); // 微动开关(如果有条件也可解耦至 IO 驱动层)

    switch (shoot_ctrl.pull_state)
    {
        case PULL_STATE_NORMAL:
            BSP_PWM_Set_Compare(&trigger_pwm, 1200);
            shoot_ctrl.PID_Pull_P.Ref += 200.0f;

            // 触发条件：微动开关触发 或 收到上位机/遥控器的触发指令
            if ((current_switch_v == GPIO_PIN_RESET && shoot_ctrl.last_switch_v == GPIO_PIN_SET) ||
                (local_shoot_cmd.trigger_single))
            {
                BSP_PWM_Set_Compare(&trigger_pwm, 600);
                shoot_ctrl.pull_delay_counter = 0;
                shoot_ctrl.pull_state = PULL_STATE_TRIGGERED;
            }
            break;

        case PULL_STATE_TRIGGERED:
            shoot_ctrl.PID_Pull_P.Ref = s_motor->DJI_3508_Pull.Angle_Infinite;
            if (++shoot_ctrl.pull_delay_counter >= 580) {
                shoot_ctrl.PID_Pull_P.Ref -= 1100000.0f;
                shoot_ctrl.feed_motor.target_pos_cnt -= 1;

                // 动态修改 PID 参数
                shoot_ctrl.PID_Pull_P.Kp = 2500;
                shoot_ctrl.pull_state = PULL_STATE_RESET;
            }
            break;

        case PULL_STATE_RESET:
            if (MATH_ABS_float(s_motor->DJI_3508_Pull.Angle_Infinite - shoot_ctrl.PID_Pull_P.Ref) < 1000.0f
                && shoot_ctrl.feed_motor.smooth_ref == final_target) {
                shoot_ctrl.PID_Pull_S.Iout = 0.0f;
                shoot_ctrl.pull_state = PULL_STATE_STOPPED;
            }
            break;

        case PULL_STATE_STOPPED:
            // 等待指令层释放开火键后，恢复 NORMAL
            if (local_shoot_cmd.mode != SHOOT_CMD_SAFE && shoot_ctrl.last_cmd_trigger == 0) {
                BSP_PWM_Set_Compare(&trigger_pwm, 1200);
                shoot_ctrl.PID_Pull_P.Kp = 4000; // 恢复高刚性参数
                shoot_ctrl.pull_state = PULL_STATE_NORMAL;
            }
            break;
    }
    shoot_ctrl.last_switch_v = current_switch_v;
    shoot_ctrl.last_cmd_trigger = local_shoot_cmd.trigger_single; // 记录上次指令状态

    // 8. 闭环输出与底层通讯
    float pull_output = 0.0f;
    if (shoot_ctrl.pull_state != PULL_STATE_STOPPED) {
        PID_Calculate(&shoot_ctrl.PID_Pull_P, s_motor->DJI_3508_Pull.Angle_Infinite, shoot_ctrl.PID_Pull_P.Ref);
        PID_Calculate(&shoot_ctrl.PID_Pull_S, s_motor->DJI_3508_Pull.Speed_now, shoot_ctrl.PID_Pull_P.Output);
        pull_output = shoot_ctrl.PID_Pull_S.Output;
    }

    DM_Motor_Send(&hfdcan3, 0x3FE, -shoot_ctrl.PID_Feed_S.Output, 0, 0, 0);
    DJI_Motor_Send(&hfdcan3, 0x200, 0, pull_output, shoot_ctrl.PID_Yaw_S.Output, 0);
}

/**
 * @brief 检测电机是否达到机械限位
 */
static uint8_t Check_Motor_Reached_Limit(const DJI_MOTOR_DATA_Typedef* motor, float target_speed, float stuck_current, uint16_t confirm_time)
{
    static uint16_t limit_check_counter = 0;
    static float Last_Angle_Infinite = 0;

    float pos_delta = MATH_ABS_float(motor->Angle_Infinite - Last_Angle_Infinite);
    float current   = MATH_ABS_float(motor->current);
    float speed     = MATH_ABS_float(motor->Speed_now);

    if (pos_delta < 10.0f && current > stuck_current && speed < MATH_ABS_float(target_speed) * 0.2f) {
        if (++limit_check_counter >= confirm_time) {
            limit_check_counter = 0;
            return 1;
        }
    } else {
        limit_check_counter = 0;
    }
    Last_Angle_Infinite = motor->Angle_Infinite;
    return 0;
}