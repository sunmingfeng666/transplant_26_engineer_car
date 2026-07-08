//
// Created by CaoKangqi on 2026/6/20.
//
#include "Chassis_Ctrl.h"
#include "All_define.h"
#include "Comm_DualBoard.h"
#include "Message_Center.h"
#include "Power_CAP.h"
#include "Power_Ctrl.h"
#include "Referee.h"
#include "System_State.h"
#include "Robot_Cmd.h"

static Chassis_Ctrl_Block_t chassis_ctrl;

Swerve_State_t S_Now;

static Subscriber_t *sys_state_sub;
static Subscriber_t *chassis_cmd_sub;
static Subscriber_t *cap_sub;
static Subscriber_t *referee_sub;

static System_State_t local_sys_state;
static Chassis_Cmd_t cmd = {0};
static Cap_t local_cap_data;
static Referee_Data_t chassis_referee;
//功率控制
static Power_Ctrl_t chassis_model;
static Motor_Power_State_t m_states[8];//底盘共8个电机
static Power_Node_t drive_nodes[4]; // 用于驱动电机
static Power_Node_t steer_nodes[4]; // 用于舵向电机
static Power_Group_t pwr_groups[2];//两个电机组

static float Chassis_Power_Arbitrator(float base_power_limit,
                                      float cur_buffer,
                                      bool boost_intent,
                                      const Cap_t *cap_data,
                                      bool *out_discharge);
/**
 * @brief 底盘控制初始化
 * @param MOTOR 底盘电机总结构体指针
 * @return uint8_t 初始化状态
 */
uint8_t Chassis_Control_Init(void)
{
    Swerve_Init(&S_Now);

    float PID_V_Param[3] = {8.0f, 0.0f, 0.0f};
    PID_Init(&chassis_ctrl.PID_Vx, 8.0f, 5.0f, PID_V_Param,0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    PID_Init(&chassis_ctrl.PID_Vy, 8.0f, 5.0f, PID_V_Param,0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);

    float PID_Vw_Param[3] = {2.0f, 0.0f, 0.0f};
    PID_Init(&chassis_ctrl.PID_Vw, 8.0f, 8.0f, PID_Vw_Param,0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);

    float PID_6020_Pos[3] = {800.0f, 0.0f, 0.0f};
    float PID_6020_Spd[3] = {85.0f,  0.0f, 0.0f};
    float PID_3508_Spd[3] = {5.0f,   0.1f,  0.0f};

    for (int i = 0; i < 4; i++)
    {
        // 6020 舵向位置环：输入弧度误差 -> 输出目标 RPM
        PID_Init(&chassis_ctrl.Steer_P[i], 250.0f,  30.0f,  PID_6020_Pos,
            0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
        // 6020 舵向速度环：输入 RPM 误差 -> 输出电流
        PID_Init(&chassis_ctrl.Steer_S[i], 16384.0f, 4000.0f, PID_6020_Spd,
            0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
        // 3508 驱动速度环：输入 RPM 误差 -> 输出电流
        PID_Init(&chassis_ctrl.Drive_S[i], 16384.0f, 3000.0f, PID_3508_Spd,
            0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    }
    Power_Ctrl_Init(&chassis_model);
    for(int i=0; i<4; i++) {
        // 配置驱动轮节点 (绑定 3508 模型)
        drive_nodes[i].state = &m_states[i];
        drive_nodes[i].model = &MODEL_M3508;

        // 配置转向轮节点 (绑定 6020 模型)
        steer_nodes[i].state = &m_states[i+4];
        steer_nodes[i].model = &MODEL_M6020;
    }
    // 配置优先级：
    // groups[0]: 低优先级，驱动轮，超功率时优先降驱动轮功率
    pwr_groups[0].nodes = drive_nodes;
    pwr_groups[0].node_count = 4;
    // groups[1]: 高优先级，舵轮转向
    pwr_groups[1].nodes = steer_nodes;
    pwr_groups[1].node_count = 4;
    //向系统下发底盘当前状态，准备中
    System_State_Report(ID_CHASSIS, STATUS_PREPARING);
    //订阅系统状态、底盘控制指令
    sys_state_sub   = SubRegister("system_state", sizeof(System_State_t));
    chassis_cmd_sub = SubRegister("chassis_cmd", sizeof(Chassis_Cmd_t));
    cap_sub         = SubRegister("supercap_data", sizeof(Cap_t));
    referee_sub     = SubRegister("referee_data", sizeof(Referee_Data_t));
    return 1;
}

/**
 * @brief 底盘控制任务
 */
void Chassis_Control_Task(const Chassis_Motor_Group_t *c_motor,
                          const IMU_Data_t *c_imu)
{
    if (c_motor == NULL || c_imu == NULL) {
        System_State_Report(ID_CHASSIS, STATUS_ERROR);
        return;
    }
    SubGetMessage(sys_state_sub, &local_sys_state);
    if (chassis_cmd_sub) SubGetMessage(chassis_cmd_sub, &cmd);
    if (cap_sub) SubGetMessage(cap_sub, &local_cap_data);
    if (referee_sub) SubGetMessage(referee_sub, &chassis_referee);
    System_State_Report(ID_CHASSIS, STATUS_RUN);

    for (int i = 0; i < 4; i++) {
        chassis_ctrl.swerve_fb.steer_angle_rad[i] = (float)c_motor->DJI_6020_Steer[i].Angle_Infinite * ENCODER_TO_RAD;
        chassis_ctrl.swerve_fb.steer_rpm[i]       = (float)c_motor->DJI_6020_Steer[i].Speed_now;
        chassis_ctrl.swerve_fb.wheel_rpm[i]       = (float)c_motor->DJI_3508_Chassis[i].Speed_now;
    }

    Swerve_Forward_Calc(&S_Now, &chassis_ctrl.swerve_fb);

    if (cmd.mode == CHASSIS_CMD_SAFE)
    {
        PID_Clear(&chassis_ctrl.PID_Vx);
        PID_Clear(&chassis_ctrl.PID_Vy);
        PID_Clear(&chassis_ctrl.PID_Vw);

        for (int i = 0; i < 4; i++) {
            PID_Clear(&chassis_ctrl.Steer_P[i]);
            PID_Clear(&chassis_ctrl.Steer_S[i]);
            PID_Clear(&chassis_ctrl.Drive_S[i]);
        }
        DJI_Motor_Send(&hfdcan1, 0x200,0,0,0,0);
        DJI_Motor_Send(&hfdcan2, 0x1FE,0,0,0,0);
    }
    else
    {
        float vx_tar = cmd.target_vx;
        float vy_tar = cmd.target_vy;
        float vw_tar = cmd.target_vw;

        PID_Calculate(&chassis_ctrl.PID_Vx, S_Now.vx, vx_tar);
        PID_Calculate(&chassis_ctrl.PID_Vy, S_Now.vy, vy_tar);
        PID_Calculate(&chassis_ctrl.PID_Vw, S_Now.vw, vw_tar);

        Swerve_Inverse_Calc(&chassis_ctrl.swerve_cmd, &S_Now,
                            chassis_ctrl.PID_Vx.Output, chassis_ctrl.PID_Vy.Output, chassis_ctrl.PID_Vw.Output,
                            vx_tar, vy_tar, vw_tar,
                            &chassis_ctrl.swerve_fb);

        for (int i = 0; i < 4; i++)
        {
            PID_Calculate(&chassis_ctrl.Steer_P[i],
                          chassis_ctrl.swerve_fb.steer_angle_rad[i],
                          chassis_ctrl.swerve_cmd.target_steer_angle_rad[i]);

            PID_Calculate(&chassis_ctrl.Steer_S[i],
                          chassis_ctrl.swerve_fb.steer_rpm[i],
                          chassis_ctrl.Steer_P[i].Output);

            PID_Calculate(&chassis_ctrl.Drive_S[i],
                          chassis_ctrl.swerve_fb.wheel_rpm[i],
                          chassis_ctrl.swerve_cmd.target_wheel_rpm[i]);

            chassis_ctrl.Drive_S[i].Output += chassis_ctrl.swerve_cmd.ff_torque_raw[i];

            chassis_ctrl.Steer_S[i].Output = MATH_Limit_float(16384, -16384, chassis_ctrl.Steer_S[i].Output);
            chassis_ctrl.Drive_S[i].Output = MATH_Limit_float(16384, -16384, chassis_ctrl.Drive_S[i].Output);
        }
    }
    if (cmd.mode != CHASSIS_CMD_SAFE)
    {
        for(int i=0; i<4; i++) {
            m_states[i].speed_rpm = chassis_ctrl.swerve_fb.wheel_rpm[i];
            m_states[i].original_cmd = chassis_ctrl.Drive_S[i].Output;

            m_states[i+4].speed_rpm = chassis_ctrl.swerve_fb.steer_rpm[i];
            m_states[i+4].original_cmd = chassis_ctrl.Steer_S[i].Output;
        }

        bool trigger_discharge = false;

        float final_limit = Chassis_Power_Arbitrator(chassis_referee.robot_status.chassis_power_limit,
                                                     chassis_referee.power_heat_data.buffer_energy,
                                                     1,&local_cap_data,&trigger_discharge);

        Power_Ctrl_Calculate(&chassis_model, final_limit, pwr_groups, 2);

        for(int i=0; i<4; i++) {
            chassis_ctrl.Drive_S[i].Output = m_states[i].limited_cmd;
            chassis_ctrl.Steer_S[i].Output = m_states[i+4].limited_cmd;
        }

        Power_Cap_Tx(&hfdcan3, 0x282,
                     trigger_discharge,
                     final_limit,
                     (uint8_t)chassis_referee.power_heat_data.buffer_energy,
                     chassis_referee.robot_status.current_HP);
    }
    //电流发送
    if (!(local_sys_state.global_mode == GLOBAL_SAFE_LOCK ||
          local_sys_state.global_mode == GLOBAL_STANDBY))
    {
        DJI_Motor_Send(&hfdcan1, 0x200,
                       (int16_t)chassis_ctrl.Drive_S[0].Output,
                       (int16_t)chassis_ctrl.Drive_S[1].Output,
                       (int16_t)chassis_ctrl.Drive_S[2].Output,
                       (int16_t)chassis_ctrl.Drive_S[3].Output);

        DJI_Motor_Send(&hfdcan2, 0x1FE,
                       (int16_t)chassis_ctrl.Steer_S[0].Output,
                       (int16_t)chassis_ctrl.Steer_S[1].Output,
                       (int16_t)chassis_ctrl.Steer_S[2].Output,
                       (int16_t)chassis_ctrl.Steer_S[3].Output);
    }

    if (!Is_Group_Online(CHASSIS)) {
        System_State_Report(ID_CHASSIS, STATUS_LOST);

    }
}

// 超级电容与缓冲能量调参宏定义
#define BUFFER_COMP_KP      2.0f    // 缓冲能量补偿的比例系数 (Kp)
#define TARGET_BUFFER       40.0f   // 目标期望缓冲能量 (J)
#define MIN_CAP_VOLTAGE     23.0f   // 超级电容最低放电阈值 (百分比)
#define RAMP_CAP_VOLTAGE    27.0f   // 斜坡衰减开始阈值 (百分比)
#define MAX_BOOST_POWER     150.0f  // 超级电容输出的最大冲刺功率 (W)

/**
 * @brief 功率策略仲裁器
 * * @param base_power_limit  裁判系统当前的基础功率上限
 * @param cur_buffer        裁判系统当前剩余的缓冲能量 (0~60J)
 * @param boost_intent      输入指令是否开启超电
 * @param cap_data          超级电容状态反馈 (包含在线状态、电量、故障码等)
 * @param out_discharge     [输出参数] 发送给超电是否开启
 * * @return float            返回最终决定的目标功率上限 (W)
 */
static float Chassis_Power_Arbitrator(float base_power_limit,
                                      float cur_buffer,
                                      bool boost_intent,
                                      const Cap_t *cap_data,
                                      bool *out_discharge)
{
    // 公式: power_comp = -Kp * (目标缓冲 - 当前缓冲)
    // 举例: 如果当前缓冲 cur_buffer 掉到了 20J (小于目标的40J)，算出来的 power_comp 就是负数 -40W。
    // 作用: 缓冲越低，自动把机器人的可用功率砍得越狠，强行让能量回充，防止扣血。
    float power_comp = -BUFFER_COMP_KP * (TARGET_BUFFER - cur_buffer);
    // 基础可用功率 = 裁判系统上限(比如45W) + 缓冲补偿(正数或负数)
    float final_target_power = base_power_limit + power_comp;
    // 超级电容离线/硬件故障保护
    // 如果电容 CAN 掉线了，或者电容上报了硬件故障码
    if (cap_data->get.offline.is_online == 0 || cap_data->get.cap_state != 0)
    {
        *out_discharge = false;
        // 电容板虽然坏了/断了，但自身依旧需要耗电，为了防止超功率，主动减去 5W 。
        return final_target_power - 5.0f;
    }
    // 在线且正常状态下的 放电/充电 逻辑
    if (boost_intent && cap_data->get.Cap_Capacity > MIN_CAP_VOLTAGE)
    {
        float boost_allowance = MAX_BOOST_POWER;
        // 斜坡衰减保护机制
        if (cap_data->get.Cap_Capacity < RAMP_CAP_VOLTAGE) {
            float ratio = (float)(cap_data->get.Cap_Capacity - MIN_CAP_VOLTAGE) /
                          (float)(RAMP_CAP_VOLTAGE - MIN_CAP_VOLTAGE);
            boost_allowance *= ratio;
        }
        // 最终软件允许的功率上限 = 基础可用功率 + 超电补偿功率
        final_target_power += boost_allowance;
        *out_discharge = true;
    }
    else
    {
        // 如果操作手没按 Shift，或者电量已经被榨干到了 23.0 以下：
        // 作用：把这 5W 功率留给超级电容，让它利用这 5W 给自己缓慢充电。
        final_target_power -= 5.0f;
        *out_discharge = false;
    }
    return final_target_power;
}