#include "Picture_Ctrl.h"
#include "Classic_Control.h"
#include "Comm_DualBoard.h"
#include "DJI_Motor.h"
#include "fdcan.h"
#include "Engineer_Limit.h"
#include "LeadScrew_Ctrl.h"  // 丝杠与图传共用 CAN3 0x200 帧，发送时带上其电流(槽 n1=ID 0x201)

#define PICTURE_LIFT_MIN 0
#define PICTURE_LIFT_MAX 1025000
#define PICTURE_TRANSVERSE_MIN (-624358)
#define PICTURE_TRANSVERSE_MAX 0
#define PICTURE_SPEED_LIMIT 7000.0f
#define PICTURE_MAX_CURRENT 10000.0f
#define PICTURE_PID_I_LIMIT 2000.0f
#define PICTURE_LIFT_DIR 1.0f
#define PICTURE_TRANSVERSE_DIR 1.0f
#define PICTURE_HOME_SPEED 1500.0f
#define PICTURE_HOME_MAX_CURRENT 4000.0f
#define PICTURE_HOME_TIMEOUT_MS 10000U
#define PICTURE_POSITION_TOLERANCE 3000
#define PICTURE_SPEED_DONE_TOLERANCE 100

typedef enum {
    PICTURE_AXIS_LIFT = 0,
    PICTURE_AXIS_TRANSVERSE = 1,
    PICTURE_AXIS_NUM = 2
} Picture_Axis_e;

typedef struct {
    Engineer_Picture_State_e state;
    PID_t pos_pid[PICTURE_AXIS_NUM];
    PID_t speed_pid[PICTURE_AXIS_NUM];
    int32_t target[PICTURE_AXIS_NUM];
    int32_t zero_offset[PICTURE_AXIS_NUM];
    uint8_t bottom_switch_last;
    uint8_t transverse_zero_switch_last;
    uint8_t axis_captured[PICTURE_AXIS_NUM];
    uint8_t homing_done;
    uint8_t fault;
    uint8_t home_action_seq;
    uint32_t homing_start_ms;
    uint8_t is_init;
} Engineer_Picture_Ctrl_t;

static Engineer_Picture_Ctrl_t picture_ctrl = {0};

static void Picture_Clear_Output(void);
static int16_t Picture_Home_Axis_Calc(Picture_Axis_e axis,
                                      const DJI_MOTOR_DATA_Typedef *motor,
                                      float target_speed,
                                      float dir);
static void Picture_Capture_Axis(Picture_Axis_e axis, const DJI_MOTOR_DATA_Typedef *motor);
static int32_t Picture_Get_Position(Picture_Axis_e axis, const DJI_MOTOR_DATA_Typedef *motor);
static uint8_t Picture_Is_Done(Picture_Axis_e axis, const DJI_MOTOR_DATA_Typedef *motor);
static int32_t Picture_Limit_Int32(int32_t value, int32_t min_value, int32_t max_value);
static float Picture_Limit_Float(float value, float min_value, float max_value);
static uint8_t Picture_Read_Lift_Bottom(void);
static uint8_t Picture_Read_Transverse_Zero(void);
static void Picture_Update_Switch_Zero(const Picture_Motor_Group_t *p_motor);
static int16_t Picture_Axis_Calc(Picture_Axis_e axis,
                                 const DJI_MOTOR_DATA_Typedef *motor,
                                 int32_t target,
                                 float dir);

uint8_t Engineer_Picture_Init(void)
{
    float pos_pid_param[3] = {0.006f, 0.0f, 0.0f};
    float speed_pid_param[3] = {2.0f, 0.05f, 0.0f};

    for (uint8_t i = 0; i < PICTURE_AXIS_NUM; i++) {
        PID_Init(&picture_ctrl.pos_pid[i],
                 PICTURE_SPEED_LIMIT,
                 0.0f,
                 pos_pid_param,
                 0.0f, 0.0f,
                 0.0f, 0.0f,
                 0,
                 NONE);
        PID_Init(&picture_ctrl.speed_pid[i],
                 PICTURE_MAX_CURRENT,
                 PICTURE_PID_I_LIMIT,
                 speed_pid_param,
                 0.0f, 0.0f,
                 0.0f, 0.0f,
                 0,
                 Integral_Limit);
    }

    picture_ctrl.home_action_seq = 0xFFU;
    picture_ctrl.state = ENGINEER_PICTURE_WAIT_FEEDBACK;
    picture_ctrl.is_init = 1U;
    return 1U;
}

void Engineer_Picture_Task(const Picture_Motor_Group_t *p_motor)
{
    if (!picture_ctrl.is_init) {
        (void)Engineer_Picture_Init();
    }

    Engineer_Limit_Update();

    // 全局前置条件1：对端失联/安全模式/急停 -> 停机
    if (p_motor == NULL ||
        !DualBoard_Picture_Is_Online() ||
        !DualBoard_Chassis_Is_Online() ||
        B2B_Chassis_Cmd.mode == DUALBOARD_CHASSIS_SAFE ||
        B2B_Picture_Cmd.mechanism_action == DUALBOARD_ACTION_STOP_ALL) {
        picture_ctrl.state = ENGINEER_PICTURE_STOPPED;
        Picture_Clear_Output();
        return;
    }

    // 人工中止/清故障：仅故障态响应，回等待反馈重建基准
    if (picture_ctrl.fault &&
        B2B_Picture_Cmd.mechanism_action == DUALBOARD_ACTION_CLEAR_FAULT) {
        picture_ctrl.fault = 0U;
        picture_ctrl.homing_done = 0U;
        picture_ctrl.state = ENGINEER_PICTURE_WAIT_FEEDBACK;
    }

    // 全局前置条件2：任一轴电机离线 -> 清捕获标志、回等待态、不跟踪(安全)
    if (!p_motor->DJI_2006_Lift.offline.is_online) {
        picture_ctrl.axis_captured[PICTURE_AXIS_LIFT] = 0U;
    }
    if (!p_motor->DJI_2006_Transverse.offline.is_online) {
        picture_ctrl.axis_captured[PICTURE_AXIS_TRANSVERSE] = 0U;
    }
    if (!p_motor->DJI_2006_Lift.offline.is_online ||
        !p_motor->DJI_2006_Transverse.offline.is_online) {
        if (picture_ctrl.state != ENGINEER_PICTURE_FAULT) {
            picture_ctrl.state = ENGINEER_PICTURE_WAIT_FEEDBACK;
        }
        Picture_Clear_Output();
        return;
    }

    // 限位触发时重建机械零点并钳位
    Picture_Update_Switch_Zero(p_motor);

    int16_t lift_out = 0;
    int16_t transverse_out = 0;

    // 归零请求(动作序号去重)：运行态收到 HOME_PICTURE 进入归零态
    if (B2B_Picture_Cmd.mechanism_action == DUALBOARD_ACTION_HOME_PICTURE &&
        picture_ctrl.state != ENGINEER_PICTURE_HOMING &&
        picture_ctrl.state != ENGINEER_PICTURE_FAULT &&
        picture_ctrl.home_action_seq != B2B_Picture_Cmd.action_seq) {
        picture_ctrl.home_action_seq = B2B_Picture_Cmd.action_seq;
        picture_ctrl.homing_done = 0U;
        picture_ctrl.homing_start_ms = HAL_GetTick();
        picture_ctrl.state = ENGINEER_PICTURE_HOMING;
    }

    switch (picture_ctrl.state) {
    case ENGINEER_PICTURE_WAIT_FEEDBACK:
        // 首次/重连后：捕获当前位置为临时零点仅用于保位，目标保持0防突跳；
        // 需发归零指令建立真实机械零点后才允许绝对目标跟踪。
        if (!picture_ctrl.axis_captured[PICTURE_AXIS_LIFT]) {
            Picture_Capture_Axis(PICTURE_AXIS_LIFT, &p_motor->DJI_2006_Lift);
        }
        if (!picture_ctrl.axis_captured[PICTURE_AXIS_TRANSVERSE]) {
            Picture_Capture_Axis(PICTURE_AXIS_TRANSVERSE, &p_motor->DJI_2006_Transverse);
        }
        lift_out = Picture_Axis_Calc(PICTURE_AXIS_LIFT, &p_motor->DJI_2006_Lift,
                                     picture_ctrl.target[PICTURE_AXIS_LIFT], PICTURE_LIFT_DIR);
        transverse_out = Picture_Axis_Calc(PICTURE_AXIS_TRANSVERSE, &p_motor->DJI_2006_Transverse,
                                           picture_ctrl.target[PICTURE_AXIS_TRANSVERSE], PICTURE_TRANSVERSE_DIR);
        break;
    case ENGINEER_PICTURE_TRACKING:
        // 跟踪 B2B 目标(已限幅)。进入条件：归零完成。
        picture_ctrl.target[PICTURE_AXIS_LIFT] =
            Picture_Limit_Int32(B2B_Picture_Cmd.picture_lift, PICTURE_LIFT_MIN, PICTURE_LIFT_MAX);
        picture_ctrl.target[PICTURE_AXIS_TRANSVERSE] =
            Picture_Limit_Int32(B2B_Picture_Cmd.picture_transverse, PICTURE_TRANSVERSE_MIN, PICTURE_TRANSVERSE_MAX);
        lift_out = Picture_Axis_Calc(PICTURE_AXIS_LIFT, &p_motor->DJI_2006_Lift,
                                     picture_ctrl.target[PICTURE_AXIS_LIFT], PICTURE_LIFT_DIR);
        transverse_out = Picture_Axis_Calc(PICTURE_AXIS_TRANSVERSE, &p_motor->DJI_2006_Transverse,
                                           picture_ctrl.target[PICTURE_AXIS_TRANSVERSE], PICTURE_TRANSVERSE_DIR);
        break;

    case ENGINEER_PICTURE_HOMING: {
        // 双轴向限位方向运行；完成条件=双限位触发；超时->故障。
        const uint8_t lift_bottom = Picture_Read_Lift_Bottom();
        const uint8_t transverse_zero = Picture_Read_Transverse_Zero();
        if ((HAL_GetTick() - picture_ctrl.homing_start_ms) > PICTURE_HOME_TIMEOUT_MS) {
            picture_ctrl.fault = 1U;
            picture_ctrl.state = ENGINEER_PICTURE_FAULT;
            Picture_Clear_Output();
            break;
        }
        if (!lift_bottom) {
            lift_out = Picture_Home_Axis_Calc(PICTURE_AXIS_LIFT, &p_motor->DJI_2006_Lift,
                                              -PICTURE_HOME_SPEED, PICTURE_LIFT_DIR);
        }
        if (!transverse_zero) {
            transverse_out = Picture_Home_Axis_Calc(PICTURE_AXIS_TRANSVERSE, &p_motor->DJI_2006_Transverse,
                                                    PICTURE_HOME_SPEED, PICTURE_TRANSVERSE_DIR);
        }
        if (lift_bottom && transverse_zero) {
            picture_ctrl.homing_done = 1U;                 // 零点已由 Update_Switch_Zero 重建
            picture_ctrl.target[PICTURE_AXIS_LIFT] = 0;
            picture_ctrl.target[PICTURE_AXIS_TRANSVERSE] = 0;
            picture_ctrl.state = ENGINEER_PICTURE_TRACKING;
        }
        break;
    }

    case ENGINEER_PICTURE_STOPPED:
        // 停机恢复：对端恢复且退出安全/急停后(已过前置条件)，回等待态重建。
        picture_ctrl.state = ENGINEER_PICTURE_WAIT_FEEDBACK;
        Picture_Clear_Output();
        return;

    case ENGINEER_PICTURE_FAULT:
        // 故障锁存，安全输出；仅 CLEAR_FAULT 可退出(见前置)。
        Picture_Clear_Output();
        return;

    default:
        Picture_Clear_Output();
        return;
    }

    if (Picture_Read_Lift_Bottom() && lift_out < 0) {
        lift_out = 0;
        PID_Clear(&picture_ctrl.pos_pid[PICTURE_AXIS_LIFT]);
        PID_Clear(&picture_ctrl.speed_pid[PICTURE_AXIS_LIFT]);
    }

    if (Picture_Read_Transverse_Zero() && transverse_out > 0) {
        transverse_out = 0;
        PID_Clear(&picture_ctrl.pos_pid[PICTURE_AXIS_TRANSVERSE]);
        PID_Clear(&picture_ctrl.speed_pid[PICTURE_AXIS_TRANSVERSE]);
    }

    // n1=丝杠(ID 0x201)、n3/n4=图传抬升/横移。丝杠电流由其独立状态机算好，此处只搭车发送。
    DJI_Motor_Send(&hfdcan3, 0x200, Engineer_LeadScrew_Get_Output(), 0, lift_out, transverse_out);
}

static void Picture_Clear_Output(void)
{
    for (uint8_t i = 0; i < PICTURE_AXIS_NUM; i++) {
        PID_Clear(&picture_ctrl.pos_pid[i]);
        PID_Clear(&picture_ctrl.speed_pid[i]);
    }
    // 图传停机时丝杠可能仍在工作：帧里保留丝杠电流，只清零图传两槽。
    DJI_Motor_Send(&hfdcan3, 0x200, Engineer_LeadScrew_Get_Output(), 0, 0, 0);
}

static int16_t Picture_Home_Axis_Calc(Picture_Axis_e axis,
                                      const DJI_MOTOR_DATA_Typedef *motor,
                                      float target_speed,
                                      float dir)
{
    const float speed = (float)motor->Speed_now * dir;
    float current = PID_Calculate(&picture_ctrl.speed_pid[axis], speed, target_speed);
    current = Picture_Limit_Float(current, -PICTURE_HOME_MAX_CURRENT, PICTURE_HOME_MAX_CURRENT);
    return (int16_t)(current * dir);
}

static void Picture_Capture_Axis(Picture_Axis_e axis, const DJI_MOTOR_DATA_Typedef *motor)
{
    // 首次收到反馈时把当前位置作为临时零点，避免上电后目标突跳；触发限位后会重建机械零点。
    picture_ctrl.zero_offset[axis] = motor->Angle_Infinite;
    picture_ctrl.target[axis] = 0;
    picture_ctrl.axis_captured[axis] = 1U;
    PID_Clear(&picture_ctrl.pos_pid[axis]);
    PID_Clear(&picture_ctrl.speed_pid[axis]);
}

static int32_t Picture_Get_Position(Picture_Axis_e axis, const DJI_MOTOR_DATA_Typedef *motor)
{
    const float dir = (axis == PICTURE_AXIS_LIFT) ? PICTURE_LIFT_DIR : PICTURE_TRANSVERSE_DIR;
    return (int32_t)(((float)motor->Angle_Infinite - (float)picture_ctrl.zero_offset[axis]) * dir);
}

static uint8_t Picture_Is_Done(Picture_Axis_e axis, const DJI_MOTOR_DATA_Typedef *motor)
{
    int32_t error = Picture_Get_Position(axis, motor) - picture_ctrl.target[axis];
    if (error < 0) error = -error;
    int16_t speed = motor->Speed_now;
    if (speed < 0) speed = (int16_t)-speed;
    return (error <= PICTURE_POSITION_TOLERANCE && speed <= PICTURE_SPEED_DONE_TOLERANCE) ? 1U : 0U;
}

static int16_t Picture_Axis_Calc(Picture_Axis_e axis,
                                 const DJI_MOTOR_DATA_Typedef *motor,
                                 int32_t target,
                                 float dir)
{
    float position = ((float)motor->Angle_Infinite - (float)picture_ctrl.zero_offset[axis]) * dir;
    float speed = (float)motor->Speed_now * dir;

    float target_speed = PID_Calculate(&picture_ctrl.pos_pid[axis], position, (float)target);
    target_speed = Picture_Limit_Float(target_speed, -PICTURE_SPEED_LIMIT, PICTURE_SPEED_LIMIT);

    float current = PID_Calculate(&picture_ctrl.speed_pid[axis], speed, target_speed);
    current = Picture_Limit_Float(current, -PICTURE_MAX_CURRENT, PICTURE_MAX_CURRENT);
    current *= dir;

    return (int16_t)current;
}

static void Picture_Update_Switch_Zero(const Picture_Motor_Group_t *p_motor)
{
    uint8_t lift_bottom = Picture_Read_Lift_Bottom();
    uint8_t transverse_zero = Picture_Read_Transverse_Zero();

    if (lift_bottom && !picture_ctrl.bottom_switch_last) {
        picture_ctrl.zero_offset[PICTURE_AXIS_LIFT] = p_motor->DJI_2006_Lift.Angle_Infinite;
        PID_Clear(&picture_ctrl.pos_pid[PICTURE_AXIS_LIFT]);
        PID_Clear(&picture_ctrl.speed_pid[PICTURE_AXIS_LIFT]);
    }
    if (lift_bottom && picture_ctrl.target[PICTURE_AXIS_LIFT] <= PICTURE_LIFT_MIN) {
        picture_ctrl.target[PICTURE_AXIS_LIFT] = PICTURE_LIFT_MIN;
    }

    if (transverse_zero && !picture_ctrl.transverse_zero_switch_last) {
        picture_ctrl.zero_offset[PICTURE_AXIS_TRANSVERSE] = p_motor->DJI_2006_Transverse.Angle_Infinite;
        PID_Clear(&picture_ctrl.pos_pid[PICTURE_AXIS_TRANSVERSE]);
        PID_Clear(&picture_ctrl.speed_pid[PICTURE_AXIS_TRANSVERSE]);
    }
    if (transverse_zero && picture_ctrl.target[PICTURE_AXIS_TRANSVERSE] >= PICTURE_TRANSVERSE_MAX) {
        picture_ctrl.target[PICTURE_AXIS_TRANSVERSE] = PICTURE_TRANSVERSE_MAX;
    }

    picture_ctrl.bottom_switch_last = lift_bottom;
    picture_ctrl.transverse_zero_switch_last = transverse_zero;
}

static uint8_t Picture_Read_Lift_Bottom(void)
{
    return Engineer_Limit_Lift_Bottom();
}

static uint8_t Picture_Read_Transverse_Zero(void)
{
    return Engineer_Limit_Transverse_Zero();
}

Engineer_Picture_Status_t Engineer_Picture_Get_Status(void)
{
    Engineer_Picture_Status_t status = {0};
    status.state = picture_ctrl.state;
    status.lift_bottom = Picture_Read_Lift_Bottom();
    status.transverse_zero = Picture_Read_Transverse_Zero();
    status.homing_active = (picture_ctrl.state == ENGINEER_PICTURE_HOMING) ? 1U : 0U;
    status.homing_done = picture_ctrl.homing_done;
    status.fault = picture_ctrl.fault;

    if (picture_motors.DJI_2006_Lift.offline.is_online &&
        picture_ctrl.axis_captured[PICTURE_AXIS_LIFT]) {
        status.lift_position = Picture_Get_Position(PICTURE_AXIS_LIFT,
                                                    &picture_motors.DJI_2006_Lift);
        status.lift_done = Picture_Is_Done(PICTURE_AXIS_LIFT,
                                          &picture_motors.DJI_2006_Lift);
    }
    if (picture_motors.DJI_2006_Transverse.offline.is_online &&
        picture_ctrl.axis_captured[PICTURE_AXIS_TRANSVERSE]) {
        status.transverse_position = Picture_Get_Position(PICTURE_AXIS_TRANSVERSE,
                                                          &picture_motors.DJI_2006_Transverse);
        status.transverse_done = Picture_Is_Done(PICTURE_AXIS_TRANSVERSE,
                                                &picture_motors.DJI_2006_Transverse);
    }
    return status;
}

static int32_t Picture_Limit_Int32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static float Picture_Limit_Float(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}
