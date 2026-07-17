//
// Created by CaoKangqi on 2026/2/23.
//
#include "Chassis_Calc.h"
#include <math.h>
#include "All_define.h"
#include "Horizon_MATH.h"

float theta_chassis;

static float ClampFloat(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

static float AbsFloat(float val)
{
    return (val < 0.0f) ? -val : val;
}

uint8_t MecanumInit(mecanumInit_typdef *mecanumInitT)
{
    // 工程车实车麦轮几何参数（与原 Chassis_Ctrl.c 内 Chassis_Resolve 逐位一致）。
    // 减速比用精确小数 0.052075f（≈1/19.2），不能写成整数除法 3591/187（会被截断成 19）。
    mecanumInitT->deceleration_ratio = 0.052075f;
    mecanumInitT->max_vw_speed       = 50000;
    mecanumInitT->max_vx_speed       = 50000;
    mecanumInitT->max_vy_speed       = 50000;
    mecanumInitT->max_wheel_ramp     = 8000;
    mecanumInitT->rotate_x_offset    = 00.0f;
    mecanumInitT->rotate_y_offset    = 00.0f;
    mecanumInitT->wheelbase          = 360;
    mecanumInitT->wheeltrack         = 380;
    mecanumInitT->wheel_perimeter    = 478;

    mecanumInitT->raid_fr = ((mecanumInitT->wheelbase + mecanumInitT->wheeltrack) / 2.0f -
                             mecanumInitT->rotate_x_offset + mecanumInitT->rotate_y_offset) / 57.3f;
    mecanumInitT->raid_fl = ((mecanumInitT->wheelbase + mecanumInitT->wheeltrack) / 2.0f -
                             mecanumInitT->rotate_x_offset - mecanumInitT->rotate_y_offset) / 57.3f;
    mecanumInitT->raid_bl = ((mecanumInitT->wheelbase + mecanumInitT->wheeltrack) / 2.0f +
                             mecanumInitT->rotate_x_offset - mecanumInitT->rotate_y_offset) / 57.3f;
    mecanumInitT->raid_br = ((mecanumInitT->wheelbase + mecanumInitT->wheeltrack) / 2.0f +
                             mecanumInitT->rotate_x_offset + mecanumInitT->rotate_y_offset) / 57.3f;

    mecanumInitT->wheel_rpm_ratio = 60.0f / (mecanumInitT->wheel_perimeter * mecanumInitT->deceleration_ratio);
    return 0;
}

// 工程车麦轮逆解算：底盘速度 -> 四个 3508 目标转速(rpm)。
// vx_temp / vy_temp 单位 mm/s，vr 单位 mrad/s（串口帧用整数传输，此处内部换算）。
// 逻辑与原 Chassis_Ctrl.c 的 Chassis_Resolve 逐位一致，不含缓启动/转向补偿。
void MecanumResolve(float *wheel_rpm, float vx_temp, float vy_temp, float vr, mecanumInit_typdef *mecanumInit_t)
{
    if (wheel_rpm == NULL || mecanumInit_t == NULL) return;

    // mrad/s -> deg/s，再乘旋转半径得到等效轮端旋转分量。
    // 57.3 是旧工程沿用的 rad->deg 近似系数，运算顺序保持不变以确保数值一致。
    float vw_deg_s = vr / 1000.0f * 57.3f;
    float rot = vw_deg_s * mecanumInit_t->raid_fr;

    // 轮序和符号沿用旧工程车底盘代码。
    // 如果实车方向反了，优先调整这里，不改串口协议。
    wheel_rpm[0] = ( vx_temp + vy_temp + rot) * mecanumInit_t->wheel_rpm_ratio;
    wheel_rpm[1] = -(-vx_temp + vy_temp - rot) * mecanumInit_t->wheel_rpm_ratio;
    wheel_rpm[2] = (-vx_temp - vy_temp + rot) * mecanumInit_t->wheel_rpm_ratio;
    wheel_rpm[3] = -( vx_temp - vy_temp + rot) * mecanumInit_t->wheel_rpm_ratio;

    // 保持运动方向不变，把四个轮子的目标转速整体压到上限内。
    float max_abs = 0.0f;
    for (uint8_t i = 0; i < 4; i++) {
        float a = AbsFloat(wheel_rpm[i]);
        if (a > max_abs) max_abs = a;
    }
    if (max_abs > (float)mecanumInit_t->max_wheel_ramp && max_abs > 0.0f) {
        float rate = (float)mecanumInit_t->max_wheel_ramp / max_abs;
        for (uint8_t i = 0; i < 4; i++) {
            wheel_rpm[i] *= rate;
        }
    }

    // 再逐轮硬限幅到 ±max_wheel_ramp，兜住浮点缩放后的微小越界。
    for (uint8_t i = 0; i < 4; i++) {
        wheel_rpm[i] = ClampFloat(wheel_rpm[i], -(float)mecanumInit_t->max_wheel_ramp,
                                  (float)mecanumInit_t->max_wheel_ramp);
    }
}

uint8_t OmniInit(OmniInit_typdef *OmniInitT)
{
    OmniInitT->wheel_perimeter = 155 * PI;
    OmniInitT->CHASSIS_DECELE_RATIO = 3591/187;
    OmniInitT->LENGTH_A = 180;
    OmniInitT->LENGTH_B = 180;
    OmniInitT->max_vx_speed = 50000;
    OmniInitT->max_vy_speed = 50000;
    OmniInitT->max_vw_speed = 50000;
    OmniInitT->max_wheel_ramp = 8000;

    OmniInitT->rotate_radius = (OmniInitT->LENGTH_A + OmniInitT->LENGTH_B) / 57.3f;
    OmniInitT->wheel_rpm_ratio = 60.0f / (OmniInitT->wheel_perimeter) * OmniInitT->CHASSIS_DECELE_RATIO;
    return 0;
}

void Omni_calc(float *wheel_rpm, float vx_temp, float vy_temp, float vr, OmniInit_typdef *OmniInit_t)
{
    float vx = ClampFloat(vx_temp, -OmniInit_t->max_vx_speed, OmniInit_t->max_vx_speed);
    float vy = ClampFloat(vy_temp, -OmniInit_t->max_vy_speed, OmniInit_t->max_vy_speed);
    float vw = ClampFloat(vr, -OmniInit_t->max_vw_speed, OmniInit_t->max_vw_speed);
    float rot = vw * OmniInit_t->rotate_radius;

    wheel_rpm[0] = (  vx + vy + rot) * OmniInit_t->wheel_rpm_ratio;
    wheel_rpm[1] = (  vx - vy + rot) * OmniInit_t->wheel_rpm_ratio;
    wheel_rpm[2] = ( -vx - vy + rot) * OmniInit_t->wheel_rpm_ratio;
    wheel_rpm[3] = ( -vx + vy + rot) * OmniInit_t->wheel_rpm_ratio;

    float max_abs = 0.0f;
    for (uint8_t i = 0; i < 4; i++) {
        float a = AbsFloat(wheel_rpm[i]);
        if (a > max_abs) max_abs = a;
    }
    if (max_abs > (float)OmniInit_t->max_wheel_ramp && max_abs > 0.0f) {
        float rate = (float)OmniInit_t->max_wheel_ramp / max_abs;
        for (uint8_t i = 0; i < 4; i++) {
            wheel_rpm[i] *= rate;
        }
    }
}


#define M3508_NM_TO_RAW ( (1.0f / (15.7647f * 0.0157f * 0.85f)) * (16384.0f / 20.0f) )

static float normalize_to_pi(float angle) {
    angle = fmodf(angle, 2.0f * PI);
    if (angle > PI)  angle -= 2.0f * PI;
    if (angle < -PI) angle += 2.0f * PI;
    return angle;
}

uint8_t Swerve_Init(Swerve_State_t *state) {
    if (state == NULL) return 1;
    __builtin_memset(state, 0, sizeof(Swerve_State_t));

    state->cfg.m = 10.5f;
    state->cfg.J = 1.0f;
    state->cfg.R = 0.24f;
    state->cfg.r = 0.06f;
    state->cfg.gear_d = 15.76f;

    state->cfg.Swerve_offset[0] = -120.0f * DEG2RAD;
    state->cfg.Swerve_offset[1] = -120.0f * DEG2RAD;
    state->cfg.Swerve_offset[2] = 60.0f * DEG2RAD;
    state->cfg.Swerve_offset[3] = 60.0f * DEG2RAD;

    state->cfg.phi[0] = 0.262f * PI;  state->cfg.phi[1] = 0.738f * PI;
    state->cfg.phi[2] = 1.262f * PI;  state->cfg.phi[3] = 1.738f * PI;

    return 0;
}

// 舵轮正解算
void Swerve_Forward_Calc(Swerve_State_t *now, const Swerve_Feedback_t *fb) {
    float b_x = 0, b_y = 0, b_w = 0;

    for (int i = 0; i < 4; i++) {
        now->wheel[i].v_wheel_now = (fb->wheel_rpm[i] * RPM_TO_RADS / now->cfg.gear_d) * now->cfg.r;

        float steer_chassis = fb->steer_angle_rad[i] - now->cfg.Swerve_offset[i];
        now->wheel[i].theta_now = normalize_to_pi(steer_chassis);

        float vix = now->wheel[i].v_wheel_now * cosf(steer_chassis);
        float viy = now->wheel[i].v_wheel_now * sinf(steer_chassis);

        b_x += now->wheel[i].v_wheel_now * cosf(now->wheel[i].theta_now);
        b_y += now->wheel[i].v_wheel_now * sinf(now->wheel[i].theta_now);
        b_w += (viy * sinf(now->cfg.phi[i]) - vix * cosf(now->cfg.phi[i])) / now->cfg.R;
    }

    now->vx = b_x / 4.0f;
    now->vy = b_y / 4.0f;
    now->vw = b_w / 4.0f;
}

// 舵轮逆解算
void Swerve_Inverse_Calc(Swerve_Command_t *cmd, Swerve_State_t *state,
                         float ax, float ay, float aw,
                         float vx, float vy, float vw,
                         const Swerve_Feedback_t *fb)
{
    state->ax_target = ax;
    state->ay_target = ay;
    state->aw_target = aw;
    state->vx_target = vx;
    state->vy_target = vy;
    state->vw_target = vw;

    for (int i = 0; i < 4; i++) {
        float vix = vx - state->cfg.R * vw * cosf(state->cfg.phi[i]);
        float viy = vy + state->cfg.R * vw * sinf(state->cfg.phi[i]);
        float v_mag = sqrtf(vix * vix + viy * viy);

        float current_theta_motor = fb->steer_angle_rad[i];
        float current_theta_chassis = current_theta_motor - state->cfg.Swerve_offset[i];

        float target_theta_raw;
        if (fabsf(v_mag) < 0.005f) {
            target_theta_raw = current_theta_chassis;
        } else {
            target_theta_raw = atan2f(viy, vix);
        }

        float diff = target_theta_raw - fmodf(current_theta_chassis, 2.0f * PI);
        while (diff >  PI) diff -= 2.0f * PI;
        while (diff < -PI) diff += 2.0f * PI;

        float speed_dir = 1.0f;
        if (fabsf(diff) > PI / 2.0f) {
            diff = (diff > 0) ? diff - PI : diff + PI;
            speed_dir = -1.0f;
        }

        state->wheel[i].theta_target = normalize_to_pi(current_theta_chassis + diff);
        state->wheel[i].v_wheel_target = speed_dir * v_mag;

        cmd->target_steer_angle_rad[i] = current_theta_motor + diff;
        cmd->target_wheel_rpm[i] = (state->wheel[i].v_wheel_target / state->cfg.r) * state->cfg.gear_d / RPM_TO_RADS;

        // 动力学前馈解算
        float F_ix = (state->cfg.m * ax - state->cfg.J * aw / state->cfg.R * cosf(state->cfg.phi[i])) / 4.0f;
        float F_iy = (state->cfg.m * ay + state->cfg.J * aw / state->cfg.R * sinf(state->cfg.phi[i])) / 4.0f;
        float F_drive = F_ix * cosf(current_theta_chassis) + F_iy * sinf(current_theta_chassis);

        state->wheel[i].ff_out = speed_dir * (F_drive * state->cfg.r) * M3508_NM_TO_RAW;

        cmd->ff_torque_raw[i] = state->wheel[i].ff_out;
    }
}

float CHASSIS_GET_MAX_TARGET(float MAX_POWER)
{
    if (MAX_POWER == 45)       return 0.03f * MAX_POWER;
    else if (MAX_POWER == 50 || MAX_POWER == 200) return 0.04f * MAX_POWER;
    else if (MAX_POWER == 55)  return 0.06f * MAX_POWER;
    else if (MAX_POWER == 60 || MAX_POWER == 75)  return 0.08f * MAX_POWER;
    else if (MAX_POWER == 65)  return 0.1f * MAX_POWER;
    else if (MAX_POWER == 70)  return 0.09f * MAX_POWER;
    else if (MAX_POWER == 80)  return 0.07f * MAX_POWER;
    else if (MAX_POWER == 90 || MAX_POWER == 100) return 0.065f * MAX_POWER;
    else                       return 0.1f * MAX_POWER;
}