//
// 工程车底盘电机执行板的麦轮底盘控制。
// 数据路径：
// USART10 命令帧 -> B2B_Chassis_Cmd -> 麦轮解算 -> 速度 PID -> FDCAN1 0x200 电流。
// 同时通过 USART10 周期性回传底盘状态，供遥控板确认双向链路。
//
#include "Chassis_Ctrl.h"

#include <math.h>

#include "All_define.h"
#include "Chassis_Calc.h"
#include "Classic_Control.h"
#include "Comm_DualBoard.h"
#include "DJI_Motor.h"
#include "fdcan.h"
#include "IMU_Task.h"
#include "Message_Center.h"
#include "Power_Ctrl.h"
#include "Referee.h"
#include "System_State.h"

//
#define ENGINEER_CHASSIS_MAX_CURRENT        12000.0f
#define ENGINEER_CHASSIS_PID_I_LIMIT        3000.0f
#define ENGINEER_CHASSIS_PID_DEFAULT_KP     5.0f
#define ENGINEER_CHASSIS_PID_DEFAULT_KI     0.1f
#define ENGINEER_CHASSIS_PID_DEFAULT_KD     0.0f
#define ENGINEER_CHASSIS_YAW_PID_KP        300.0f
#define ENGINEER_CHASSIS_YAW_PID_KI          0.0f
#define ENGINEER_CHASSIS_YAW_PID_KD          0.0f
#define ENGINEER_CHASSIS_YAW_MAX_CORRECTION_MRAD_S 1200.0f
#define ENGINEER_CHASSIS_YAW_PID_I_LIMIT_MRAD_S     300.0f
#define ENGINEER_CHASSIS_YAW_MIN_TARGET_MRAD_S       50.0f
#define ENGINEER_CHASSIS_YAW_DIRECTION_CHECK_RAD_S    0.3f
#define ENGINEER_CHASSIS_GYRO_Z_SIGN                  1.0f
#define ENGINEER_CHASSIS_FALLBACK_POWER_W   140.0f
#define ENGINEER_CHASSIS_BUFFER_TARGET_J    40.0f
#define ENGINEER_CHASSIS_BUFFER_KP           2.0f
#define ENGINEER_CHASSIS_MAX_BUFFER_J        60U
#define ENGINEER_CHASSIS_MAX_REF_POWER_W    500U

typedef struct {
    PID_t speed_pid[4];
    PID_t yaw_rate_pid;
    float target_rpm[4];   // 目标转速顺序：0x201、0x202、0x203、0x204。
    float target_vw_rad_s;          // 小陀螺目标角速度，供 Ozone 观察。
    float gyro_z_rad_s;             // BMI088 逻辑 Z 轴角速度，供 Ozone 观察。
    float yaw_correction_mrad_s;    // 角速度外环修正量。
    float resolved_vw_mrad_s;       // 送入麦轮解算的最终角速度。
    DualBoard_Chassis_Feedback_Status_e feedback_status;
    int16_t feedback_error;
    uint8_t feedback_motor_bits;   // 最近一次记录的 3508 在线位图
    uint8_t yaw_loop_active;       // 1：本周期角速度外环生效。
    uint8_t yaw_direction_fault;   // 1：目标与陀螺仪方向相反，本周期退回开环。
    uint8_t is_init;
} Engineer_Chassis_Ctrl_t;


// 在线调 PID：直接改 chassis_ctrl.speed_pid[i].Kp/Ki/Kd，下一控制周期(1kHz)即生效。
Engineer_Chassis_Ctrl_t chassis_ctrl __attribute__((used));
volatile Engineer_Chassis_Speed_t Engineer_Chassis_Speed[4] = {0};
static mecanumInit_typdef chassis_mecanum;   // 麦轮解算参数，Init 时由 MecanumInit 填充。
static Power_Ctrl_t chassis_power_model;
static Motor_Power_State_t chassis_power_states[4];
static Power_Node_t chassis_power_nodes[4];
static Power_Group_t chassis_power_group;
static Subscriber_t *referee_sub;
static Subscriber_t *imu_sub;
static Referee_Data_t chassis_referee;
static IMU_Data_t chassis_imu;

static void Chassis_Clear_Output(void);
static float Chassis_Resolve_Yaw_Rate(float target_vw_mrad_s);
static void Chassis_Record_Feedback(const Chassis_Motor_Group_t *c_motor,
                                    DualBoard_Chassis_Feedback_Status_e status,
                                    int16_t error_code);
static void Chassis_Update_Speed_Monitor(const Chassis_Motor_Group_t *c_motor);
static uint8_t Chassis_Get_Motor_Online_Bits(const Chassis_Motor_Group_t *c_motor);
static float Chassis_Get_Allowed_Power(const Referee_Data_t *referee, uint8_t *using_referee_power);
static float Limit_Finite(float value, float min_value, float max_value, float fallback);
static float Limit_Float(float value, float min_value, float max_value);

uint8_t Engineer_Chassis_Init(void)
{
    float pid_param[3] = {
        ENGINEER_CHASSIS_PID_DEFAULT_KP,
        ENGINEER_CHASSIS_PID_DEFAULT_KI,
        ENGINEER_CHASSIS_PID_DEFAULT_KD,
    };
    float yaw_pid_param[3] = {
        ENGINEER_CHASSIS_YAW_PID_KP,
        ENGINEER_CHASSIS_YAW_PID_KI,
        ENGINEER_CHASSIS_YAW_PID_KD,
    };

    Power_Ctrl_Init(&chassis_power_model);
    referee_sub = SubRegister("referee_data", sizeof(Referee_Data_t));
    imu_sub = SubRegister("imu_data", sizeof(IMU_Data_t));
    for (uint8_t i = 0; i < 4; i++) {
        PID_Init(&chassis_ctrl.speed_pid[i],
                 ENGINEER_CHASSIS_MAX_CURRENT,
                 ENGINEER_CHASSIS_PID_I_LIMIT,
                 pid_param,
                 0.0f, 0.0f,
                 0.0f, 0.0f,
                 0,
                 Integral_Limit);

        // 四个 3508 作为同一个功率组，统一按比例削减电流，保持底盘运动方向。
        chassis_power_nodes[i].state = &chassis_power_states[i];
        chassis_power_nodes[i].model = &MODEL_M3508;
    }
    // 外环输入为 rad/s，输出定义为 mrad/s 修正量，与板间底盘角速度命令单位一致。
    PID_Init(&chassis_ctrl.yaw_rate_pid,
             ENGINEER_CHASSIS_YAW_MAX_CORRECTION_MRAD_S,
             ENGINEER_CHASSIS_YAW_PID_I_LIMIT_MRAD_S,
             yaw_pid_param,
             0.0f, 0.0f,
             0.0f, 0.0f,
             0,
             Integral_Limit);
    PID_Clear(&chassis_ctrl.yaw_rate_pid);
    chassis_power_group.nodes = chassis_power_nodes;
    chassis_power_group.node_count = 4U;

    // 麦轮几何参数与解算统一交给 Chassis_Calc 的 MecanumInit/MecanumResolve。
    (void)MecanumInit(&chassis_mecanum);
    chassis_ctrl.is_init = 1U;

    System_State_Report(ID_CHASSIS, STATUS_PREPARING);
    return 1U;
}

void Engineer_Chassis_Task(const Chassis_Motor_Group_t *c_motor)
{

    //保护层
    if (c_motor == NULL) {
        // 没有电机反馈时闭环不可信，直接清零输出。
        Chassis_Clear_Output();
        Chassis_Update_Speed_Monitor(NULL);
        Chassis_Record_Feedback(c_motor, DUALBOARD_FB_ERROR, 1);
        System_State_Report(ID_CHASSIS, STATUS_ERROR);
        return;
    }

    if (!chassis_ctrl.is_init) {
        (void)Engineer_Chassis_Init();
    }

    if (!DualBoard_Chassis_Is_Online() || B2B_Chassis_Cmd.mode == DUALBOARD_CHASSIS_SAFE) {
        // 串口超时或显式安全模式：清 PID 积分并输出 0 电流。
        Chassis_Clear_Output();
        Chassis_Update_Speed_Monitor(c_motor);
        Chassis_Record_Feedback(c_motor, DUALBOARD_FB_LOST, 0);
        System_State_Report(ID_CHASSIS, STATUS_LOST);
        return;
    }

    if (imu_sub != NULL) {
        SubGetMessage(imu_sub, &chassis_imu);
    }

    // 小陀螺模式先做 BMI088 角速度外环；其他模式保持原始角速度命令。
    const float resolved_vw_mrad_s = Chassis_Resolve_Yaw_Rate(B2B_Chassis_Cmd.vw_mrad_s);

    // 麦轮逆解算：vw 用 mrad/s，MecanumResolve 内部换算并做限幅。
    MecanumResolve(chassis_ctrl.target_rpm,
                   B2B_Chassis_Cmd.vx_mm_s,
                   B2B_Chassis_Cmd.vy_mm_s,
                   resolved_vw_mrad_s,
                   &chassis_mecanum);
    Chassis_Update_Speed_Monitor(c_motor);

    float raw_out[4] = {0.0f};
    int16_t out[4] = {0};
    if (referee_sub != NULL) {
        SubGetMessage(referee_sub, &chassis_referee);
    }

    for (uint8_t i = 0; i < 4; i++) {
        raw_out[i] = PID_Calculate(&chassis_ctrl.speed_pid[i],
                                   (float)c_motor->DJI_3508_Chassis[i].Speed_now,
                                   chassis_ctrl.target_rpm[i]);
        chassis_power_states[i].speed_rpm = (float)c_motor->DJI_3508_Chassis[i].Speed_now;
        chassis_power_states[i].original_cmd = raw_out[i];
    }

    uint8_t using_referee_power = 0U;
    const float allowed_power = Chassis_Get_Allowed_Power(&chassis_referee, &using_referee_power);
    Power_Ctrl_Calculate(&chassis_power_model, allowed_power, &chassis_power_group, 1U);

    for (uint8_t i = 0; i < 4; i++) {
        const float limited = Limit_Finite(chassis_power_states[i].limited_cmd,
                                           -ENGINEER_CHASSIS_MAX_CURRENT,
                                           ENGINEER_CHASSIS_MAX_CURRENT,
                                           0.0f);
        out[i] = (int16_t)limited;
    }

    DJI_Motor_Send(&hfdcan1, 0x200, out[0], out[1], out[2], out[3]);
    Chassis_Record_Feedback(c_motor, DUALBOARD_FB_RUN, 0);
    System_State_Report(ID_CHASSIS, STATUS_RUN);
}

static void Chassis_Clear_Output(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        // 清 PID，避免遥控板重连后积分残留导致电机突然动作。
        PID_Clear(&chassis_ctrl.speed_pid[i]);
        chassis_ctrl.target_rpm[i] = 0.0f;
    }
    PID_Clear(&chassis_ctrl.yaw_rate_pid);
    chassis_ctrl.target_vw_rad_s = 0.0f;
    chassis_ctrl.gyro_z_rad_s = 0.0f;
    chassis_ctrl.yaw_correction_mrad_s = 0.0f;
    chassis_ctrl.resolved_vw_mrad_s = 0.0f;
    chassis_ctrl.yaw_loop_active = 0U;
    chassis_ctrl.yaw_direction_fault = 0U;
    DJI_Motor_Send(&hfdcan1, 0x200, 0, 0, 0, 0);
}

// 仅在小陀螺模式使用 BMI088 Z 轴角速度修正；任何异常均退回原始命令。
static float Chassis_Resolve_Yaw_Rate(float target_vw_mrad_s)
{
    const float target_vw_rad_s = target_vw_mrad_s * 0.001f;
    const float gyro_z_rad_s = chassis_imu.gyro[2] * ENGINEER_CHASSIS_GYRO_Z_SIGN;

    chassis_ctrl.target_vw_rad_s = target_vw_rad_s;
    chassis_ctrl.gyro_z_rad_s = gyro_z_rad_s;
    chassis_ctrl.yaw_correction_mrad_s = 0.0f;
    chassis_ctrl.resolved_vw_mrad_s = target_vw_mrad_s;
    chassis_ctrl.yaw_loop_active = 0U;
    chassis_ctrl.yaw_direction_fault = 0U;

    const uint8_t imu_valid = (uint8_t)(imu_ctrl_state == FUSION_RUN &&
                                        imu_ctrl_flag.fusion_enabled &&
                                        isfinite(gyro_z_rad_s));
    const uint8_t spin_requested = (uint8_t)(B2B_Chassis_Cmd.mode == DUALBOARD_CHASSIS_SPIN &&
                                             isfinite(target_vw_mrad_s) &&
                                             fabsf(target_vw_mrad_s) > ENGINEER_CHASSIS_YAW_MIN_TARGET_MRAD_S);

    if (!imu_valid || !spin_requested) {
        PID_Clear(&chassis_ctrl.yaw_rate_pid);
        return target_vw_mrad_s;
    }

    // 陀螺仪方向配置错误时，闭环会成为正反馈；检测到反号后立即退回开环。
    if ((target_vw_rad_s * gyro_z_rad_s) < 0.0f &&
        fabsf(gyro_z_rad_s) > ENGINEER_CHASSIS_YAW_DIRECTION_CHECK_RAD_S) {
        PID_Clear(&chassis_ctrl.yaw_rate_pid);
        chassis_ctrl.yaw_direction_fault = 1U;
        return target_vw_mrad_s;
    }

    const float correction_mrad_s = PID_Calculate(&chassis_ctrl.yaw_rate_pid,
                                                   gyro_z_rad_s,
                                                   target_vw_rad_s);
    if (!isfinite(correction_mrad_s)) {
        PID_Clear(&chassis_ctrl.yaw_rate_pid);
        return target_vw_mrad_s;
    }

    chassis_ctrl.yaw_correction_mrad_s = correction_mrad_s;
    chassis_ctrl.resolved_vw_mrad_s = target_vw_mrad_s + correction_mrad_s;
    chassis_ctrl.yaw_loop_active = 1U;
    return chassis_ctrl.resolved_vw_mrad_s;
}

static void Chassis_Update_Speed_Monitor(const Chassis_Motor_Group_t *c_motor)
{
    for (uint8_t i = 0U; i < 4U; i++) {
        Engineer_Chassis_Speed[i].Target_rpm = chassis_ctrl.target_rpm[i];
        if (c_motor != NULL) {
            Engineer_Chassis_Speed[i].Speed_now =
                (float)c_motor->DJI_3508_Chassis[i].Speed_now;
        }
    }
}

//底盘功率
static float Chassis_Get_Allowed_Power(const Referee_Data_t *referee, uint8_t *using_referee_power)
{
    const uint32_t now = HAL_GetTick();
    if (using_referee_power != NULL) *using_referee_power = 0U;
    if (referee == NULL || !referee->offline.is_online ||
        (referee->valid_flags & REFEREE_VALID_POWER_DATA) != REFEREE_VALID_POWER_DATA ||
        (now - referee->robot_status_tick) > REFEREE_OFFLINE_TIME ||
        (now - referee->power_heat_tick) > REFEREE_OFFLINE_TIME ||
        referee->robot_status.chassis_power_limit == 0U ||
        referee->robot_status.chassis_power_limit > ENGINEER_CHASSIS_MAX_REF_POWER_W ||
        referee->power_heat_data.buffer_energy > ENGINEER_CHASSIS_MAX_BUFFER_J) {
        // 裁判掉线、关键帧未收齐或数值异常时使用固定 45W，避免零值和脏数据直接控制底盘。
        return ENGINEER_CHASSIS_FALLBACK_POWER_W;
    }

    if (using_referee_power != NULL) *using_referee_power = 1U;
    const float allowed_power = (float)referee->robot_status.chassis_power_limit
                              + ENGINEER_CHASSIS_BUFFER_KP
                              * ((float)referee->power_heat_data.buffer_energy
                                 - ENGINEER_CHASSIS_BUFFER_TARGET_J);
    // 缓冲能量耗尽时允许降到 0W；严禁产生负功率上限。
    return (allowed_power > 0.0f) ? allowed_power : 0.0f;
}

//双板
// 仅记录底盘运行状态，不直接发串口。
// 串口上报统一由 Feedback 模块以固定节奏、中断方式完成，避免阻塞 1kHz 控制环。
static void Chassis_Record_Feedback(const Chassis_Motor_Group_t *c_motor,
                                    DualBoard_Chassis_Feedback_Status_e status,
                                    int16_t error_code)
{
    chassis_ctrl.feedback_status = status;
    chassis_ctrl.feedback_error = error_code;
    chassis_ctrl.feedback_motor_bits = Chassis_Get_Motor_Online_Bits(c_motor);
}

Engineer_Chassis_Feedback_t Engineer_Chassis_Get_Feedback(void)
{
    Engineer_Chassis_Feedback_t fb;
    fb.status = chassis_ctrl.feedback_status;
    fb.motor_online_bits = chassis_ctrl.feedback_motor_bits;
    fb.error_code = chassis_ctrl.feedback_error;
    return fb;
}

//判断电机是否在线
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
