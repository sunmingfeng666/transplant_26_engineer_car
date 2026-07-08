//
// Created by CaoKangqi on 2026/2/23.
//

#ifndef G4_FRAMEWORK_CHASSIS_CALC_H
#define G4_FRAMEWORK_CHASSIS_CALC_H
#include <stdint.h>

typedef struct
{
    float wheel_perimeter;    /* 轮的周长（mm）*/
    float wheeltrack;         /* 轮距（mm）*/
    float wheelbase;          /* 轴距（mm）*/
    float rotate_x_offset;    /* 相对于底盘中心的x轴旋转偏移量(mm) */
    float rotate_y_offset;    /* 相对于底盘中心的y轴旋转偏移量(mm) */
    float deceleration_ratio; /* 电机减速比 */
    int max_vx_speed;         /* 底盘的x轴的最大速度(mm/s) */
    int max_vy_speed;         /* 底盘的y轴的最大速度(mm/s) */
    int max_vw_speed;         /* 底盘的自转的最大速度(degree/s) */
    int max_wheel_ramp;       /* 3508最大转速 */
    float raid_fr;            // 右前
    float raid_fl;            // 左前
    float raid_bl;            // 左后
    float raid_br;            // 右后
    float wheel_rpm_ratio;    // 用来将速度转化成转每分钟
} mecanumInit_typdef;

typedef struct
{
    float wheel_perimeter;    /* 轮的周长（mm）*/
    float CHASSIS_DECELE_RATIO; /* 电机减速比 */
    float LENGTH_A;         /* 底盘长的一半（mm）*/
    float LENGTH_B;          /* 底盘宽的一半（mm）*/
    float rotate_radius;     /* (A+B)/57.3, 将角速度(deg/s)转换为线速度(mm/s) */
    float wheel_rpm_ratio;   /* 线速度(mm/s) -> 轮速(rpm) */
    int max_vx_speed;        /* 底盘x轴最大速度(mm/s) */
    int max_vy_speed;        /* 底盘y轴最大速度(mm/s) */
    int max_vw_speed;        /* 底盘自转最大速度(deg/s) */
    int max_wheel_ramp;      /* 3508最大转速 */
} OmniInit_typdef;

extern OmniInit_typdef OmniInit_t;

uint8_t MecanumInit(mecanumInit_typdef *mecanumInitT);
void MecanumResolve(float *wheel_rpm, float vx_temp, float vy_temp, float vr, mecanumInit_typdef *mecanumInit_t);

uint8_t OmniInit(OmniInit_typdef *OmniInitT);
void Omni_calc(float *wheel_rpm, float vx_temp, float vy_temp, float vr, OmniInit_typdef *OmniInit_t);


/* ==================== 舵轮 ==================== */

// 舵轮物理常量结构体
typedef struct {
    float m;                // 底盘质量 (kg)
    float J;                // 转动惯量 (kg*m^2)
    float R;                // 旋转半径 (m)
    float r;                // 轮子半径 (m)
    float Swerve_offset[4]; // 舵轮零点偏角
    float phi[4];           // 轮子安装方位角 (rad)
    float gear_d;           // 驱动电机减速比
} Swerve_Cfg_t;

// 实时单轮调试/物理状态
typedef struct {
    float theta_now;       // 归一化±π的舵轮实际角度 (rad)
    float theta_target;    // 归一化±π的舵轮目标角度 (rad)
    float v_wheel_now;     // 单轮实际线速度 (m/s)
    float v_wheel_target;  // 单轮目标线速度 (m/s)
    float ff_out;          // 单轮前馈输出原始值
} Swerve_Wheel_Debug_t;

// 底盘整体状态结构体
typedef struct {
    Swerve_Cfg_t cfg;

    // 底盘核心速度状态
    float vx;              // 底盘x轴实际速度 (m/s)
    float vy;              // 底盘y轴实际速度 (m/s)
    float vw;              // 底盘实际角速度 (rad/s)

    // 底盘目标指令
    float vx_target;       // 底盘x轴目标速度 (m/s)
    float vy_target;       // 底盘y轴目标速度 (m/s)
    float vw_target;       // 底盘目标角速度 (rad/s)
    float ax_target;       // 底盘x轴目标加速度 (m/s²)
    float ay_target;       // 底盘y轴目标加速度 (m/s²)
    float aw_target;       // 底盘目标角加速度 (rad/s²)

    Swerve_Wheel_Debug_t wheel[4];
} Swerve_State_t;

// 反馈数据，输入给解算器
typedef struct {
    float steer_angle_rad[4];  // 舵轮当前连续绝对角度 (rad)
    float steer_rpm[4];
    float wheel_rpm[4];        // 驱动轮当前转速 (RPM)
} Swerve_Feedback_t;

// 解算器输出的指令数据
typedef struct {
    float target_steer_angle_rad[4]; // 目标舵向角 (rad)
    float target_wheel_rpm[4];       // 目标驱动轮转速 (RPM)
    float ff_torque_raw[4];          // 前馈控制量
} Swerve_Command_t;

uint8_t Swerve_Init(Swerve_State_t *state);

void Swerve_Forward_Calc(Swerve_State_t *now, const Swerve_Feedback_t *fb);

void Swerve_Inverse_Calc(Swerve_Command_t *cmd, Swerve_State_t *state,
                         float ax, float ay, float aw,
                         float vx, float vy, float vw,
                         const Swerve_Feedback_t *fb);

float CHASSIS_GET_MAX_TARGET(float MAX_POWER);

#endif //G4_FRAMEWORK_CHASSIS_CALC_H