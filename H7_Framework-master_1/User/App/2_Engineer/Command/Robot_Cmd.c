//
// 工程车底盘电机执行板命令中心。
// 第一版底盘板不直接读取遥控器。
// 本地 App 主题保持安全状态，真实底盘运动来自 Comm_DualBoard。
//
#include "Robot_Cmd.h"
#include "Message_Center.h"
#include "System_State.h"

static Subscriber_t *sys_state_sub;

static Publisher_t *chassis_cmd_pub;
static Publisher_t *gimbal_cmd_pub;
static Publisher_t *shoot_cmd_pub;

static System_State_t cmd_sys_state;
static Chassis_Cmd_t chassis_cmd = {0};
static Gimbal_Cmd_t gimbal_cmd = {0};
static Shoot_Cmd_t shoot_cmd = {0};

static void Cmd_Handle_Safe_Mode(void);

void Robot_Cmd_Init(void)
{
    sys_state_sub = SubRegister("system_state", sizeof(System_State_t));

    chassis_cmd_pub = PubRegister("chassis_cmd", &chassis_cmd, sizeof(Chassis_Cmd_t));
    gimbal_cmd_pub  = PubRegister("gimbal_cmd", &gimbal_cmd, sizeof(Gimbal_Cmd_t));
    shoot_cmd_pub   = PubRegister("shoot_cmd", &shoot_cmd, sizeof(Shoot_Cmd_t));
}

void Robot_Cmd_Update(void)
{
    if (sys_state_sub) SubGetMessage(sys_state_sub, &cmd_sys_state);

    // 保持框架内部主题有效，但底盘板本地不生成运动指令。
    Cmd_Handle_Safe_Mode();

    PubPushMessage(chassis_cmd_pub, &chassis_cmd);
    PubPushMessage(gimbal_cmd_pub, &gimbal_cmd);
    PubPushMessage(shoot_cmd_pub, &shoot_cmd);
}

static void Cmd_Handle_Safe_Mode(void)
{
    chassis_cmd.mode = CHASSIS_CMD_SAFE;
    chassis_cmd.target_vx = 0.0f;
    chassis_cmd.target_vy = 0.0f;
    chassis_cmd.target_vw = 0.0f;
    chassis_cmd.offset_angle = 0.0f;

    gimbal_cmd.mode = GIMBAL_CMD_SAFE;
    gimbal_cmd.target_pitch = 0.0f;
    gimbal_cmd.target_yaw = 0.0f;

    shoot_cmd.mode = SHOOT_CMD_SAFE;
    shoot_cmd.friction_rpm = 0.0f;
    shoot_cmd.trigger_single = false;
    shoot_cmd.trigger_auto = false;
    shoot_cmd.bullet_speed = 0U;
}
