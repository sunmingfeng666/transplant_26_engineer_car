#include "LeadScrew_Ctrl.h"
#include "Classic_Control.h"
#include "Comm_DualBoard.h"
#include "DJI_Motor.h"
#include "Engineer_Limit.h"

// 行程：下限位为机械零点，向上为正，[0, MAX]。上限位为硬顶。
#define LEADSCREW_TRAVEL_MIN 0
#define LEADSCREW_TRAVEL_MAX 900000
#define LEADSCREW_DIR 1.0f
#define LEADSCREW_SPEED_LIMIT 7000.0f
#define LEADSCREW_MAX_CURRENT 16000.0f
#define LEADSCREW_PID_I_LIMIT 2000.0f
#define LEADSCREW_HOME_SPEED 4000.0f
#define LEADSCREW_HOME_MAX_CURRENT 6000.0f
#define LEADSCREW_DIRECT_SPEED_LIMIT 2000.0f
#define LEADSCREW_HOME_TIMEOUT_MS 12000U
#define LEADSCREW_POSITION_TOLERANCE 3000
#define LEADSCREW_SPEED_DONE_TOLERANCE 100

typedef struct {
    Engineer_LeadScrew_State_e state;
    PID_t pos_pid;
    PID_t speed_pid;
    int32_t target;
    int32_t zero_offset;
    int32_t cmd_target;          // 命令层写入的目标(中断/异步)
    uint8_t up_switch_last;
    uint8_t down_switch_last;
    uint8_t captured;
    uint8_t homing_done;
    uint8_t fault;
    uint8_t home_action_seq;
    uint32_t homing_start_ms;
    uint8_t is_init;
    int16_t output;              // 供图传任务发送的电流
} Engineer_LeadScrew_Ctrl_t;

static Engineer_LeadScrew_Ctrl_t ls_ctrl = {0};
static uint8_t s_direct_active = 0U;
volatile Engineer_LeadScrew_Debug_t Engineer_LeadScrew_Debug = {0};

static int32_t LeadScrew_Limit_Int32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float LeadScrew_Limit_Float(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int32_t LeadScrew_Get_Position(const DJI_MOTOR_DATA_Typedef *motor)
{
    return (int32_t)(((float)motor->Angle_Infinite - (float)ls_ctrl.zero_offset) * LEADSCREW_DIR);
}

static void LeadScrew_Clear_Output(void)
{
    PID_Clear(&ls_ctrl.pos_pid);
    PID_Clear(&ls_ctrl.speed_pid);
    ls_ctrl.output = 0;
}

// 首次收反馈：把当前位置设为临时零点，目标=0 防突跳；归零后重建真实零点。
static void LeadScrew_Capture(const DJI_MOTOR_DATA_Typedef *motor)
{
    ls_ctrl.zero_offset = motor->Angle_Infinite;
    ls_ctrl.target = 0;
    ls_ctrl.captured = 1U;
    LeadScrew_Clear_Output();
}

// 位置→速度→电流双环。老代码值：位置环 Kp0.12/Ki2e-5/Kd1e-4，速度环 Kp1.8。
static int16_t LeadScrew_Axis_Calc(const DJI_MOTOR_DATA_Typedef *motor, int32_t target)
{
    float position = ((float)motor->Angle_Infinite - (float)ls_ctrl.zero_offset) * LEADSCREW_DIR;
    float speed = (float)motor->Speed_now * LEADSCREW_DIR;

    float target_speed = PID_Calculate(&ls_ctrl.pos_pid, position, (float)target);
    target_speed = LeadScrew_Limit_Float(target_speed, -LEADSCREW_SPEED_LIMIT, LEADSCREW_SPEED_LIMIT);

    float current = PID_Calculate(&ls_ctrl.speed_pid, speed, target_speed);
    current = LeadScrew_Limit_Float(current, -LEADSCREW_MAX_CURRENT, LEADSCREW_MAX_CURRENT);
    current *= LEADSCREW_DIR;
    return (int16_t)current;
}

// 归零：仅速度环，向下限位方向恒速运行。
static int16_t LeadScrew_Home_Calc(const DJI_MOTOR_DATA_Typedef *motor, float target_speed)
{
    float speed = (float)motor->Speed_now * LEADSCREW_DIR;
    float current = PID_Calculate(&ls_ctrl.speed_pid, speed, target_speed);
    current = LeadScrew_Limit_Float(current, -LEADSCREW_HOME_MAX_CURRENT, LEADSCREW_HOME_MAX_CURRENT);
    return (int16_t)(current * LEADSCREW_DIR);
}

static void LeadScrew_Update_Debug(const DJI_MOTOR_DATA_Typedef *motor)
{
    Engineer_LeadScrew_Debug.up_limit = Engineer_Limit_LeadScrew_Up();
    Engineer_LeadScrew_Debug.down_limit = Engineer_Limit_LeadScrew_Down();
    Engineer_LeadScrew_Debug.output = ls_ctrl.output;

    if (motor != NULL) {
        Engineer_LeadScrew_Debug.motor_online = motor->offline.is_online ? 1U : 0U;
        Engineer_LeadScrew_Debug.encoder = motor->Angle_Infinite;
        Engineer_LeadScrew_Debug.speed = motor->Speed_now;
    } else {
        Engineer_LeadScrew_Debug.motor_online = 0U;
        Engineer_LeadScrew_Debug.encoder = 0;
        Engineer_LeadScrew_Debug.speed = 0;
    }
}

uint8_t Engineer_LeadScrew_Init(void)
{
    float pos_pid_param[3] = {0.12f, 0.00002f, 0.0001f};
    float speed_pid_param[3] = {1.8f, 0.0f, 0.0f};

    PID_Init(&ls_ctrl.pos_pid, LEADSCREW_SPEED_LIMIT, LEADSCREW_PID_I_LIMIT,
             pos_pid_param, 0.0f, 0.0f, 0.7f, 0.7f, 0, Integral_Limit);
    PID_Init(&ls_ctrl.speed_pid, LEADSCREW_MAX_CURRENT, LEADSCREW_PID_I_LIMIT,
             speed_pid_param, 0.0f, 0.0f, 0.7f, 0.7f, 0, Integral_Limit);

    ls_ctrl.home_action_seq = 0xFFU;
    ls_ctrl.state = ENGINEER_LEADSCREW_WAIT_FEEDBACK;
    ls_ctrl.is_init = 1U;
    return 1U;
}

// 下限位=机械零点，触发上升沿重建零点；两端限位到达时向内钳目标。
static void LeadScrew_Update_Switch_Zero(const DJI_MOTOR_DATA_Typedef *motor)
{
    uint8_t up = Engineer_Limit_LeadScrew_Up();
    uint8_t down = Engineer_Limit_LeadScrew_Down();

    if (down && !ls_ctrl.down_switch_last) {
        ls_ctrl.zero_offset = motor->Angle_Infinite;
        PID_Clear(&ls_ctrl.pos_pid);
        PID_Clear(&ls_ctrl.speed_pid);
    }
    if (down && ls_ctrl.target <= LEADSCREW_TRAVEL_MIN) {
        ls_ctrl.target = LEADSCREW_TRAVEL_MIN;
    }
    if (up && ls_ctrl.target >= LEADSCREW_TRAVEL_MAX) {
        ls_ctrl.target = LEADSCREW_TRAVEL_MAX;
    }

    ls_ctrl.up_switch_last = up;
    ls_ctrl.down_switch_last = down;
}

void Engineer_LeadScrew_Task(const Picture_Motor_Group_t *p_motor)
{
    if (!ls_ctrl.is_init) {
        (void)Engineer_LeadScrew_Init();
    }

    const DJI_MOTOR_DATA_Typedef *motor =
        (p_motor != NULL) ? &p_motor->DJI_3508_LeadScrew : NULL;

    /* Direct debug speed control bypasses the normal state machine. */
    if (Engineer_LeadScrew_Debug.direct_enable != 0U) {
        s_direct_active = 1U;
        if (motor == NULL || !motor->offline.is_online) {
            Engineer_LeadScrew_Debug.direct_speed = 0.0f;
            LeadScrew_Clear_Output();
            LeadScrew_Update_Debug(motor);
            return;
        }

        LeadScrew_Update_Switch_Zero(motor);
        float target_speed = LeadScrew_Limit_Float(
            Engineer_LeadScrew_Debug.direct_speed,
            -LEADSCREW_DIRECT_SPEED_LIMIT,
            LEADSCREW_DIRECT_SPEED_LIMIT);
        int16_t out = LeadScrew_Home_Calc(motor, target_speed);

        if (Engineer_Limit_LeadScrew_Down() && out < 0) {
            out = 0;
            PID_Clear(&ls_ctrl.speed_pid);
        }
        if (Engineer_Limit_LeadScrew_Up() && out > 0) {
            out = 0;
            PID_Clear(&ls_ctrl.speed_pid);
        }

        ls_ctrl.output = out;
        LeadScrew_Update_Debug(motor);
        return;
    }

    if (s_direct_active != 0U) {
        s_direct_active = 0U;
        ls_ctrl.captured = 0U;
        ls_ctrl.state = ENGINEER_LEADSCREW_WAIT_FEEDBACK;
        LeadScrew_Clear_Output();
    }

    // 前置1：对端失联/安全/急停 -> 停机安全输出。
    if (p_motor == NULL ||
        !DualBoard_Picture_Is_Online() ||
        !DualBoard_Chassis_Is_Online() ||
        B2B_Chassis_Cmd.mode == DUALBOARD_CHASSIS_SAFE ||
        B2B_Picture_Cmd.mechanism_action == DUALBOARD_ACTION_STOP_ALL) {
        ls_ctrl.state = ENGINEER_LEADSCREW_STOPPED;
        LeadScrew_Clear_Output();
        return;
    }

    // 人工清故障：仅故障态响应，回等待反馈重建基准。
    if (ls_ctrl.fault &&
        B2B_Picture_Cmd.mechanism_action == DUALBOARD_ACTION_CLEAR_FAULT) {
        ls_ctrl.fault = 0U;
        ls_ctrl.homing_done = 0U;
        ls_ctrl.state = ENGINEER_LEADSCREW_WAIT_FEEDBACK;
    }

    // 前置2：电机离线 -> 清捕获、回等待、安全输出。
    if (!motor->offline.is_online) {
        ls_ctrl.captured = 0U;
        if (ls_ctrl.state != ENGINEER_LEADSCREW_FAULT) {
            ls_ctrl.state = ENGINEER_LEADSCREW_WAIT_FEEDBACK;
        }
        LeadScrew_Clear_Output();
        return;
    }

    LeadScrew_Update_Switch_Zero(motor);
    int16_t out = 0;

    // 归零请求(动作序号去重)：复用整车 HOME_PICTURE 动作一并归零丝杠。
    if (B2B_Picture_Cmd.mechanism_action == DUALBOARD_ACTION_HOME_PICTURE &&
        ls_ctrl.state != ENGINEER_LEADSCREW_HOMING &&
        ls_ctrl.state != ENGINEER_LEADSCREW_FAULT &&
        ls_ctrl.home_action_seq != B2B_Picture_Cmd.action_seq) {
        ls_ctrl.home_action_seq = B2B_Picture_Cmd.action_seq;
        ls_ctrl.homing_done = 0U;
        ls_ctrl.homing_start_ms = HAL_GetTick();
        ls_ctrl.state = ENGINEER_LEADSCREW_HOMING;
    }

    switch (ls_ctrl.state) {
    case ENGINEER_LEADSCREW_WAIT_FEEDBACK:
        // 首次/重连：捕获当前位置为临时零点保位，目标保持0防突跳。
        if (!ls_ctrl.captured) {
            LeadScrew_Capture(motor);
        }
        out = LeadScrew_Axis_Calc(motor, ls_ctrl.target);
        break;

    case ENGINEER_LEADSCREW_TRACKING:
        // 跟踪命令层目标(已限幅)。进入条件：归零完成。
        ls_ctrl.target = LeadScrew_Limit_Int32(ls_ctrl.cmd_target,
                                               LEADSCREW_TRAVEL_MIN, LEADSCREW_TRAVEL_MAX);
        out = LeadScrew_Axis_Calc(motor, ls_ctrl.target);
        break;

    case ENGINEER_LEADSCREW_HOMING: {
        // 向下限位方向恒速运行；完成=下限位触发；超时->故障。
        const uint8_t down = Engineer_Limit_LeadScrew_Down();
        if ((HAL_GetTick() - ls_ctrl.homing_start_ms) > LEADSCREW_HOME_TIMEOUT_MS) {
            ls_ctrl.fault = 1U;
            ls_ctrl.state = ENGINEER_LEADSCREW_FAULT;
            LeadScrew_Clear_Output();
            break;
        }
        if (!down) {
            out = LeadScrew_Home_Calc(motor, -LEADSCREW_HOME_SPEED);
        } else {
            ls_ctrl.homing_done = 1U;           // 零点已由 Update_Switch_Zero 重建
            ls_ctrl.target = LEADSCREW_TRAVEL_MIN;
            ls_ctrl.cmd_target = LEADSCREW_TRAVEL_MIN;
            ls_ctrl.state = ENGINEER_LEADSCREW_TRACKING;
        }
        break;
    }

    case ENGINEER_LEADSCREW_STOPPED:
        // 停机恢复：前置条件已过 -> 回等待态重建。
        ls_ctrl.state = ENGINEER_LEADSCREW_WAIT_FEEDBACK;
        LeadScrew_Clear_Output();
        return;

    case ENGINEER_LEADSCREW_FAULT:
        // 故障锁存，安全输出；仅 CLEAR_FAULT 可退出(见前置)。
        LeadScrew_Clear_Output();
        return;

    default:
        LeadScrew_Clear_Output();
        return;
    }

    // 到达限位方向仍出力则截零并清积分，防堵转。
    if (Engineer_Limit_LeadScrew_Down() && out < 0) {
        out = 0;
        PID_Clear(&ls_ctrl.pos_pid);
        PID_Clear(&ls_ctrl.speed_pid);
    }
    if (Engineer_Limit_LeadScrew_Up() && out > 0) {
        out = 0;
        PID_Clear(&ls_ctrl.pos_pid);
        PID_Clear(&ls_ctrl.speed_pid);
    }

    ls_ctrl.output = out;
}

void Engineer_LeadScrew_Set_Target(int32_t target)
{
    ls_ctrl.cmd_target = LeadScrew_Limit_Int32(target, LEADSCREW_TRAVEL_MIN, LEADSCREW_TRAVEL_MAX);
}

int16_t Engineer_LeadScrew_Get_Output(void)
{
    return ls_ctrl.output;
}

Engineer_LeadScrew_Status_t Engineer_LeadScrew_Get_Status(void)
{
    Engineer_LeadScrew_Status_t status = {0};
    status.state = ls_ctrl.state;
    status.up_limit = Engineer_Limit_LeadScrew_Up();
    status.down_limit = Engineer_Limit_LeadScrew_Down();
    status.homing_active = (ls_ctrl.state == ENGINEER_LEADSCREW_HOMING) ? 1U : 0U;
    status.homing_done = ls_ctrl.homing_done;
    status.fault = ls_ctrl.fault;

    if (picture_motors.DJI_3508_LeadScrew.offline.is_online && ls_ctrl.captured) {
        status.position = LeadScrew_Get_Position(&picture_motors.DJI_3508_LeadScrew);
        int32_t err = status.position - ls_ctrl.target;
        if (err < 0) err = -err;
        int16_t spd = picture_motors.DJI_3508_LeadScrew.Speed_now;
        if (spd < 0) spd = (int16_t)-spd;
        status.done = (err <= LEADSCREW_POSITION_TOLERANCE &&
                       spd <= LEADSCREW_SPEED_DONE_TOLERANCE) ? 1U : 0U;
    }
    return status;
}
