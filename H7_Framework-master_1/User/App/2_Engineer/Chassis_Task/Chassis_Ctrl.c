//
// 工程车底盘电机执行板的麦轮底盘控制。
// 数据路径：
// USART10 命令帧 -> B2B_Chassis_Cmd -> 麦轮解算 -> 速度 PID -> FDCAN1 0x200 电流。
// 同时通过 USART10 周期性回传底盘状态，供遥控板确认双向链路。
//
#include "Chassis_Ctrl.h"

#include <math.h>

#include "Classic_Control.h"
#include "Comm_DualBoard.h"
#include "DJI_Motor.h"
#include "fdcan.h"
#include "System_State.h"
#include "usart.h"

#define ENGINEER_CHASSIS_WHEELBASE_MM       360.0f
#define ENGINEER_CHASSIS_WHEELTRACK_MM      380.0f
#define ENGINEER_CHASSIS_WHEEL_PERIMETER_MM 478.0f
#define ENGINEER_CHASSIS_DECEL_RATIO        0.052075f
#define ENGINEER_CHASSIS_MAX_WHEEL_RPM      8000.0f
#define ENGINEER_CHASSIS_MAX_CURRENT        12000.0f
#define ENGINEER_CHASSIS_PID_I_LIMIT        3000.0f
#define ENGINEER_CHASSIS_FEEDBACK_PERIOD_MS 20U
#define ENGINEER_CHASSIS_PID_DEFAULT_KP     5.0f
#define ENGINEER_CHASSIS_PID_DEFAULT_KI     0.1f
#define ENGINEER_CHASSIS_PID_DEFAULT_KD     0.0f
#define ENGINEER_CHASSIS_PID_MAX_KP         100.0f
#define ENGINEER_CHASSIS_PID_MAX_KI         10.0f
#define ENGINEER_CHASSIS_PID_MAX_KD         10.0f

typedef struct {
    volatile float kp[4];
    volatile float ki[4];
    volatile float kd[4];
    volatile float max_out[4];
    volatile float i_limit[4];
    volatile float target_rpm[4];
    volatile float speed_rpm[4];
    volatile float output_current[4];
} Chassis_PID_Readback_t;

typedef struct {
    PID_t speed_pid[4];
    float rotate_radius;   // 把底盘角速度换算成等效轮端线速度。
    float wheel_rpm_ratio; // 把轮端线速度(mm/s)换算成 3508 电机侧 rpm。
    float target_rpm[4];   // 目标转速顺序：0x201、0x202、0x203、0x204。
    DualBoard_Chassis_Feedback_Status_e feedback_status;
    int16_t feedback_error;
    uint32_t last_feedback_ms;
    uint8_t is_init;
} Engineer_Chassis_Ctrl_t;

static Engineer_Chassis_Ctrl_t chassis_ctrl;

volatile float Chassis_PID_Kp[4] __attribute__((used)) = {0};
volatile float Chassis_PID_Ki[4] __attribute__((used)) = {0};
volatile float Chassis_PID_Kd[4] __attribute__((used)) = {0};
volatile float Chassis_PID_MaxOut[4] __attribute__((used)) = {0};
volatile float Chassis_PID_ILimit[4] __attribute__((used)) = {0};
volatile uint8_t Chassis_PID_Clear __attribute__((used)) = 0U;
volatile Chassis_PID_Readback_t Chassis_PID_Readback __attribute__((used)) = {0};

static void Chassis_Clear_Output(void);
static void Chassis_Send_Feedback(const Chassis_Motor_Group_t *c_motor,
                                  DualBoard_Chassis_Feedback_Status_e status,
                                  int16_t error_code);
static uint8_t Chassis_Get_Motor_Online_Bits(const Chassis_Motor_Group_t *c_motor);
static void Chassis_Resolve(float vx_mm_s, float vy_mm_s, float vw_mrad_s, float *wheel_rpm);
static void Chassis_PID_Debug_Init_Defaults(void);
static void Chassis_PID_Debug_Sync(void);
static void Chassis_PID_Debug_Update_Readback(const Chassis_Motor_Group_t *c_motor, const int16_t out[4]);
static float Limit_Finite(float value, float min_value, float max_value, float fallback);
static float Limit_Float(float value, float min_value, float max_value);
static float Abs_Float(float value);

uint8_t Engineer_Chassis_Init(void)
{
    float pid_param[3] = {
        ENGINEER_CHASSIS_PID_DEFAULT_KP,
        ENGINEER_CHASSIS_PID_DEFAULT_KI,
        ENGINEER_CHASSIS_PID_DEFAULT_KD,
    };

    Chassis_PID_Debug_Init_Defaults();
    for (uint8_t i = 0; i < 4; i++) {
        // 使用新框架 PID 工具做 3508 速度环。
        // PID 输出直接作为 DJI 电机 0x200 电流命令发送。
        PID_Init(&chassis_ctrl.speed_pid[i],
                 ENGINEER_CHASSIS_MAX_CURRENT,
                 ENGINEER_CHASSIS_PID_I_LIMIT,
                 pid_param,
                 0.0f, 0.0f,
                 0.0f, 0.0f,
                 0,
                 Integral_Limit);
    }

    // 第一版移植保持和旧工程一致的麦轮几何参数。
    // 57.3 是旧工程沿用的 rad->deg 近似系数。
    chassis_ctrl.rotate_radius = ((ENGINEER_CHASSIS_WHEELBASE_MM + ENGINEER_CHASSIS_WHEELTRACK_MM) / 2.0f) / 57.3f;
    chassis_ctrl.wheel_rpm_ratio = 60.0f / (ENGINEER_CHASSIS_WHEEL_PERIMETER_MM * ENGINEER_CHASSIS_DECEL_RATIO);
    chassis_ctrl.is_init = 1U;

    System_State_Report(ID_CHASSIS, STATUS_PREPARING);
    return 1U;
}

void Engineer_Chassis_Task(const Chassis_Motor_Group_t *c_motor)
{
    if (c_motor == NULL) {
        // 没有电机反馈时闭环不可信，直接清零输出。
        Chassis_Clear_Output();
        Chassis_Send_Feedback(c_motor, DUALBOARD_FB_ERROR, 1);
        System_State_Report(ID_CHASSIS, STATUS_ERROR);
        return;
    }

    if (!chassis_ctrl.is_init) {
        (void)Engineer_Chassis_Init();
    }

    if (!DualBoard_Chassis_Is_Online() || B2B_Chassis_Cmd.mode == DUALBOARD_CHASSIS_SAFE) {
        // 串口超时或显式安全模式：清 PID 积分并输出 0 电流。
        Chassis_Clear_Output();
        Chassis_Send_Feedback(c_motor, DUALBOARD_FB_LOST, 0);
        System_State_Report(ID_CHASSIS, STATUS_LOST);
        return;
    }

    Chassis_Resolve(B2B_Chassis_Cmd.vx_mm_s,
                    B2B_Chassis_Cmd.vy_mm_s,
                    B2B_Chassis_Cmd.vw_mrad_s,
                    chassis_ctrl.target_rpm);

    int16_t out[4] = {0};
    Chassis_PID_Debug_Sync();
    for (uint8_t i = 0; i < 4; i++) {
        out[i] = (int16_t)PID_Calculate(&chassis_ctrl.speed_pid[i],
                                        (float)c_motor->DJI_3508_Chassis[i].Speed_now,
                                        chassis_ctrl.target_rpm[i]);
    }
    Chassis_PID_Debug_Update_Readback(c_motor, out);

    DJI_Motor_Send(&hfdcan1, 0x200, out[0], out[1], out[2], out[3]);
    Chassis_Send_Feedback(c_motor, DUALBOARD_FB_RUN, 0);
    System_State_Report(ID_CHASSIS, STATUS_RUN);
}

static void Chassis_Clear_Output(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        // 清 PID，避免遥控板重连后积分残留导致电机突然动作。
        PID_Clear(&chassis_ctrl.speed_pid[i]);
        chassis_ctrl.target_rpm[i] = 0.0f;
    }
    DJI_Motor_Send(&hfdcan1, 0x200, 0, 0, 0, 0);
}

static void Chassis_PID_Debug_Init_Defaults(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        Chassis_PID_Kp[i] = ENGINEER_CHASSIS_PID_DEFAULT_KP;
        Chassis_PID_Ki[i] = ENGINEER_CHASSIS_PID_DEFAULT_KI;
        Chassis_PID_Kd[i] = ENGINEER_CHASSIS_PID_DEFAULT_KD;
        Chassis_PID_MaxOut[i] = ENGINEER_CHASSIS_MAX_CURRENT;
        Chassis_PID_ILimit[i] = ENGINEER_CHASSIS_PID_I_LIMIT;
    }
    Chassis_PID_Clear = 0U;
}

static void Chassis_PID_Debug_Sync(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        float kpid[3];

        kpid[0] = Limit_Finite(Chassis_PID_Kp[i], 0.0f, ENGINEER_CHASSIS_PID_MAX_KP,
                               ENGINEER_CHASSIS_PID_DEFAULT_KP);
        kpid[1] = Limit_Finite(Chassis_PID_Ki[i], 0.0f, ENGINEER_CHASSIS_PID_MAX_KI,
                               ENGINEER_CHASSIS_PID_DEFAULT_KI);
        kpid[2] = Limit_Finite(Chassis_PID_Kd[i], 0.0f, ENGINEER_CHASSIS_PID_MAX_KD,
                               ENGINEER_CHASSIS_PID_DEFAULT_KD);

        Chassis_PID_Kp[i] = kpid[0];
        Chassis_PID_Ki[i] = kpid[1];
        Chassis_PID_Kd[i] = kpid[2];
        Chassis_PID_MaxOut[i] = Limit_Finite(Chassis_PID_MaxOut[i], 0.0f,
                                             ENGINEER_CHASSIS_MAX_CURRENT,
                                             ENGINEER_CHASSIS_MAX_CURRENT);
        Chassis_PID_ILimit[i] = Limit_Finite(Chassis_PID_ILimit[i], 0.0f,
                                             ENGINEER_CHASSIS_PID_I_LIMIT,
                                             ENGINEER_CHASSIS_PID_I_LIMIT);

        PID_set(&chassis_ctrl.speed_pid[i], kpid);
        chassis_ctrl.speed_pid[i].MaxOut = Chassis_PID_MaxOut[i];
        chassis_ctrl.speed_pid[i].IntegralLimit = Chassis_PID_ILimit[i];
        if (Chassis_PID_Clear != 0U) {
            PID_Clear(&chassis_ctrl.speed_pid[i]);
        }
    }

    if (Chassis_PID_Clear != 0U) {
        Chassis_PID_Clear = 0U;
    }
}

static void Chassis_PID_Debug_Update_Readback(const Chassis_Motor_Group_t *c_motor, const int16_t out[4])
{
    for (uint8_t i = 0; i < 4; i++) {
        Chassis_PID_Readback.kp[i] = chassis_ctrl.speed_pid[i].Kp;
        Chassis_PID_Readback.ki[i] = chassis_ctrl.speed_pid[i].Ki;
        Chassis_PID_Readback.kd[i] = chassis_ctrl.speed_pid[i].Kd;
        Chassis_PID_Readback.max_out[i] = chassis_ctrl.speed_pid[i].MaxOut;
        Chassis_PID_Readback.i_limit[i] = chassis_ctrl.speed_pid[i].IntegralLimit;
        Chassis_PID_Readback.target_rpm[i] = chassis_ctrl.target_rpm[i];
        Chassis_PID_Readback.speed_rpm[i] = (c_motor != NULL) ?
            (float)c_motor->DJI_3508_Chassis[i].Speed_now : 0.0f;
        Chassis_PID_Readback.output_current[i] = (out != NULL) ? (float)out[i] : 0.0f;
    }
}

static void Chassis_Send_Feedback(const Chassis_Motor_Group_t *c_motor,
                                  DualBoard_Chassis_Feedback_Status_e status,
                                  int16_t error_code)
{
    uint32_t now = HAL_GetTick();
    // 底盘控制是 1kHz，反馈 20ms 一次即可，避免串口发送挤占控制周期。
    if ((now - chassis_ctrl.last_feedback_ms) < ENGINEER_CHASSIS_FEEDBACK_PERIOD_MS) return;
    chassis_ctrl.last_feedback_ms = now;
    chassis_ctrl.feedback_status = status;
    chassis_ctrl.feedback_error = error_code;

    (void)DualBoard_Send_Chassis_Feedback(&huart10,
                                          status,
                                          Chassis_Get_Motor_Online_Bits(c_motor),
                                          error_code);
}

static uint8_t Chassis_Get_Motor_Online_Bits(const Chassis_Motor_Group_t *c_motor)
{
    if (c_motor == NULL) return 0U;

    uint8_t bits = 0U;
    for (uint8_t i = 0; i < 4; i++) {
        // bit0~bit3 分别代表 0x201~0x204 四个 3508 是否在线。
        if (c_motor->DJI_3508_Chassis[i].offline.is_online) {
            bits |= (uint8_t)(1U << i);
        }
    }
    return bits;
}

static void Chassis_Resolve(float vx_mm_s, float vy_mm_s, float vw_mrad_s, float *wheel_rpm)
{
    if (wheel_rpm == NULL) return;

    // 发送端用 mrad/s，是为了让串口帧只传整数。
    float vw_deg_s = vw_mrad_s / 1000.0f * 57.3f;
    float rot = vw_deg_s * chassis_ctrl.rotate_radius;

    // 轮序和符号沿用旧工程车底盘代码。
    // 如果实车方向反了，优先调整这里，不改串口协议。
    wheel_rpm[0] = ( vx_mm_s + vy_mm_s + rot) * chassis_ctrl.wheel_rpm_ratio;
    wheel_rpm[1] = (-vx_mm_s + vy_mm_s + rot) * chassis_ctrl.wheel_rpm_ratio;
    wheel_rpm[2] = (-vx_mm_s - vy_mm_s + rot) * chassis_ctrl.wheel_rpm_ratio;
    wheel_rpm[3] = ( vx_mm_s - vy_mm_s + rot) * chassis_ctrl.wheel_rpm_ratio;

    // 保持运动方向不变，把四个轮子的目标转速整体压到上限内。
    float max_abs = 0.0f;
    for (uint8_t i = 0; i < 4; i++) {
        float abs_value = Abs_Float(wheel_rpm[i]);
        if (abs_value > max_abs) max_abs = abs_value;
    }

    if (max_abs > ENGINEER_CHASSIS_MAX_WHEEL_RPM && max_abs > 0.0f) {
        float scale = ENGINEER_CHASSIS_MAX_WHEEL_RPM / max_abs;
        for (uint8_t i = 0; i < 4; i++) {
            wheel_rpm[i] *= scale;
        }
    }

    for (uint8_t i = 0; i < 4; i++) {
        wheel_rpm[i] = Limit_Float(wheel_rpm[i], -ENGINEER_CHASSIS_MAX_WHEEL_RPM, ENGINEER_CHASSIS_MAX_WHEEL_RPM);
    }
}

static float Limit_Float(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static float Limit_Finite(float value, float min_value, float max_value, float fallback)
{
    if (!isfinite(value)) value = fallback;
    return Limit_Float(value, min_value, max_value);
}

static float Abs_Float(float value)
{
    return (value < 0.0f) ? -value : value;
}
