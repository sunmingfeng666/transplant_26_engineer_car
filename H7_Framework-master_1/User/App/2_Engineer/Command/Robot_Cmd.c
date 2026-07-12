//
// 工程车底盘电机执行板命令中心。
// 第一版底盘板不直接读取遥控器。
// 本地 App 主题保持安全状态，真实底盘运动来自 Comm_DualBoard。
//
#include "Robot_Cmd.h"
#include "Comm_DualBoard.h"
#include "Message_Center.h"
#include "System_State.h"

static Subscriber_t *sys_state_sub;

static Publisher_t *chassis_cmd_pub;

static System_State_t cmd_sys_state;
static Chassis_Cmd_t chassis_cmd = {0};

static void Cmd_Handle_Safe_Mode(void);
static void Cmd_Parse_DualBoard_Frames(void);

void Robot_Cmd_Init(void)
{
    sys_state_sub = SubRegister("system_state", sizeof(System_State_t));

    chassis_cmd_pub = PubRegister("chassis_cmd", &chassis_cmd, sizeof(Chassis_Cmd_t));
}

void Robot_Cmd_Update(void)
{
    if (sys_state_sub) SubGetMessage(sys_state_sub, &cmd_sys_state);

    // 解析通信层交付的双板命令帧，写入 B2B_Chassis_Cmd / B2B_Picture_Cmd。
    Cmd_Parse_DualBoard_Frames();

    // 保持框架内部主题有效，但底盘板本地不生成运动指令。
    Cmd_Handle_Safe_Mode();

    PubPushMessage(chassis_cmd_pub, &chassis_cmd);
}

static void Cmd_Handle_Safe_Mode(void)
{
    chassis_cmd.mode = CHASSIS_CMD_SAFE;
    chassis_cmd.target_vx = 0.0f;
    chassis_cmd.target_vy = 0.0f;
    chassis_cmd.target_vw = 0.0f;
    chassis_cmd.offset_angle = 0.0f;

}

// 把原始底盘运动字段写入共享命令结构体（原 Comm 层 Update_Chassis_Cmd 逻辑）。
static void Cmd_Apply_Chassis(uint8_t mode, int16_t vx, int16_t vy, int16_t vw, uint8_t seq)
{
    if (mode > DUALBOARD_CHASSIS_SPIN) return;
    B2B_Chassis_Cmd.mode = (DualBoard_Chassis_Mode_e)mode;
    B2B_Chassis_Cmd.vx_mm_s = (float)vx;
    B2B_Chassis_Cmd.vy_mm_s = (float)vy;
    B2B_Chassis_Cmd.vw_mrad_s = (float)vw;
    B2B_Chassis_Cmd.last_seq = seq;
    B2B_Chassis_Cmd.last_update_ms = HAL_GetTick();
    B2B_Chassis_Cmd.is_online = true;
}

// 取通信层交付的原始命令帧并映射到业务命令结构体（含业务范围检查）。
static void Cmd_Parse_DualBoard_Frames(void)
{
    uint8_t buf[DUALBOARD_ENGINEER_FRAME_LEN];
    uint16_t len = 0;

    // 收件箱为单缓冲，取到的即最新一帧；无新帧直接返回。
    if (!DualBoard_Take_Cmd_Frame(buf, sizeof(buf), &len)) return;

    if (len == sizeof(B2B_Engineer_Frame_t)) {
        const B2B_Engineer_Frame_t *f = (const B2B_Engineer_Frame_t *)buf;
        if (f->mode > DUALBOARD_CHASSIS_SPIN) return;
        if (f->mechanism_action > DUALBOARD_ACTION_CLEAR_FAULT) return;
        if (f->store_slot > 3U) return;

        Cmd_Apply_Chassis(f->mode, f->vx_mm_s, f->vy_mm_s, f->vw_mrad_s, f->seq);

        B2B_Picture_Cmd.picture_lift = f->picture_lift;
        B2B_Picture_Cmd.picture_transverse = f->picture_transverse;
        B2B_Picture_Cmd.mechanism_action = (DualBoard_Mechanism_Action_e)f->mechanism_action;
        B2B_Picture_Cmd.store_slot = f->store_slot;
        B2B_Picture_Cmd.action_seq = f->action_seq;
        B2B_Picture_Cmd.ui_flags = f->ui_flags;
        B2B_Picture_Cmd.last_seq = f->seq;
        B2B_Picture_Cmd.last_update_ms = HAL_GetTick();
        B2B_Picture_Cmd.is_online = true;
        return;
    }

    if (len == sizeof(B2B_Chassis_Frame_t)) {
        const B2B_Chassis_Frame_t *f = (const B2B_Chassis_Frame_t *)buf;
        // 旧版底盘命令帧；反馈帧(mode==0x81)本板不消费。
        if (f->mode == DUALBOARD_FRAME_TYPE_FEEDBACK) return;
        Cmd_Apply_Chassis(f->mode, f->vx_mm_s, f->vy_mm_s, f->vw_mrad_s, f->seq);
    }
}
