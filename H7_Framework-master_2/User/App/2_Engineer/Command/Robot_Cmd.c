#include "Robot_Cmd.h"

#include "Message_Center.h"
#include "System_State.h"
#include "DBUS.h"
#include "VT13.h"
#include "Comm_DualBoard.h"
#include "Picture_Servo.h"
#include "Arm_OneClick.h"
#include "Arm_JointController.h"
#include "usart.h"
#include <string.h>

#define RC_ROCKER_XY_COEF 0.004f
#define RC_ROCKER_VW_COEF 0.02f
#define KB_WASD_COEF 1.0f

#define PICTURE_LIFT_MIN 0
#define PICTURE_LIFT_MAX 1025000
#define PICTURE_TRANSVERSE_MIN (-624358)
#define PICTURE_TRANSVERSE_MAX 0
#define PICTURE_LIFT_STEP 3500
#define PICTURE_TRANSVERSE_STEP 3600
#define PICTURE_SERVO_STEP_US 10

// DBUS 纯摇杆测机构：摇杆死区 + 满杆行程(660-死区)，用于把摇杆偏移换算成每周期步进。
#define RC_MECH_DEADZONE 40
#define RC_STICK_RANGE   620

static Subscriber_t *sys_state_sub;
static Subscriber_t *dbus_sub;
static Subscriber_t *vt13_sub;

static Publisher_t *chassis_cmd_pub;
static Publisher_t *picture_cmd_pub;

static System_State_t cmd_sys_state;
static DBUS_Typedef dbus_data;
static VT13_Typedef vt13_data;

static Chassis_Cmd_t chassis_cmd = {0};
static Picture_Cmd_t picture_cmd = {
    .lift = 0,
    .transverse = 0,
    .yaw_us = PICTURE_SERVO_YAW_DEFAULT_US,
    .pitch_us = PICTURE_SERVO_PITCH_DEFAULT_US,
    .enable = 0U,
};

volatile Chassis_Cmd_t Chassis_Debug_Readback __attribute__((used)) = {0};
volatile uint8_t Chassis_Debug_RemoteOnline __attribute__((used)) = 0U;

// 板2板间通信调试快照：集中观察解算结果，不重复保存遥控器原始帧。
typedef struct {
    uint32_t last_update_ms;
    uint8_t remote_online_bits;
    uint8_t last_send_result;
    DBUS_Typedef dbus;
    VT13_Typedef vt13;
    Chassis_Cmd_t chassis_cmd;
    Picture_Cmd_t picture_cmd;
    B2B_Aux_Command_t auxiliary;
    B2B_Engineer_Feedback_t board1_feedback;
} B2B_Board2_Debug_t;

volatile B2B_Board2_Debug_t B2B_Board2_Debug __attribute__((used)) = {0};

static void Cmd_Handle_Safe_Mode(void);
static void Cmd_Parse_DualBoard_Feedback(void);
static void Cmd_Update_Remote_Ctrl(void);
static void Cmd_Update_Mouse_Key(void);
static void Cmd_Update_Picture_Ctrl(void);
static void Cmd_Update_Mechanism_Test(void);
static void Cmd_Update_Stick_Mechanism(void);
static int32_t Stick_Delta(int16_t ch, int32_t step);
static void Cmd_Update_OneClick_Request(void);
static void Cmd_Update_Chassis_Debug(void);
static void Cmd_DualBoard_Sync(void);
static int32_t Cmd_Limit_Int32(int32_t value, int32_t min_value, int32_t max_value);
static uint16_t Cmd_Limit_Servo_Us(int32_t value);
static uint8_t KeyActive(uint8_t key_state);
static uint8_t Cmd_KeyActive(uint8_t dbus_key, uint8_t vt13_key);
static uint8_t Cmd_VT13KeyboardActive(void);

static uint8_t mechanism_action_seq;
static uint8_t last_action_keys;
static uint8_t last_vt_action_keys;
static uint8_t oneclick_override_last;

// 手动机构测试(不走一键)：Ctrl 图传模式下用键盘直接驱动 board1 归零/存矿。
static uint8_t mechanism_test_active;                 // 本周期是否处于手动机构测试模式
static DualBoard_Mechanism_Action_e manual_mech_action = DUALBOARD_ACTION_HOLD;
static uint8_t manual_store_slot = 3U;                // 初值3，首次 Ctrl+E 自增后从槽0开始循环
static uint8_t last_home_key;                         // Ctrl+Q 归零边沿
static uint8_t last_store_key;                        // Ctrl+E 存矿边沿
static uint8_t last_mech_s2;                           // S1=3 机构模式的上周期 S2 挡位
static uint8_t last_vt_home_trigger;                   // VT13 挡位1扳机归零边沿

void Robot_Cmd_Init(void)
{
    sys_state_sub = SubRegister("system_state", sizeof(System_State_t));
    dbus_sub = SubRegister("dbus_data", sizeof(DBUS_Typedef));
    vt13_sub = SubRegister("vt13_data", sizeof(VT13_Typedef));

    chassis_cmd_pub = PubRegister("chassis_cmd", &chassis_cmd, sizeof(Chassis_Cmd_t));
    picture_cmd_pub = PubRegister("picture_cmd", &picture_cmd, sizeof(Picture_Cmd_t));

    Picture_Servo_Init();
    mechanism_action_seq = 0U;
    last_action_keys = 0U;
    last_vt_action_keys = 0U;
    oneclick_override_last = 0U;

    mechanism_test_active = 0U;
    manual_mech_action = DUALBOARD_ACTION_HOLD;
    manual_store_slot = 3U;
    last_home_key = 0U;
    last_store_key = 0U;
    last_mech_s2 = 0U;
    last_vt_home_trigger = 0U;
}

void Robot_Cmd_Update(void)
{
    if (sys_state_sub) SubGetMessage(sys_state_sub, &cmd_sys_state);
    if (dbus_sub) SubGetMessage(dbus_sub, &dbus_data);
    if (vt13_sub) SubGetMessage(vt13_sub, &vt13_data);

    // 解析通信层交付的整车反馈帧，写入 B2B_Engineer_Feedback，供下方决策闭环使用。
    Cmd_Parse_DualBoard_Feedback();

    System_State_Report_Remote(vt13_data.offline.is_online || dbus_data.offline.is_online);
    {
        const uint8_t required_online = DUALBOARD_MECHANISM_LIFT_ONLINE |
                                        DUALBOARD_MECHANISM_TRANSVERSE_ONLINE |
                                        DUALBOARD_MECHANISM_STORE_ONLINE;
        const uint8_t peer_online = DualBoard_Engineer_Feedback_Is_Online() ? 1U : 0U;
        const uint8_t peer_ready = (peer_online &&
                                    (B2B_Engineer_Feedback.mechanism_online_bits & required_online) == required_online &&
                                    (B2B_Engineer_Feedback.action_bits & DUALBOARD_ACTION_FAULT) == 0U) ? 1U : 0U;
        const uint8_t normal_done_bits = DUALBOARD_ACTION_LIFT_DONE |
                                         DUALBOARD_ACTION_TRANSVERSE_DONE |
                                         DUALBOARD_ACTION_STORE_DONE;
        const uint8_t completion_valid =
            (((B2B_Engineer_Feedback.action_bits & normal_done_bits) == normal_done_bits) ||
             (B2B_Engineer_Feedback.action_bits & DUALBOARD_ACTION_HOMING_DONE)) ? 1U : 0U;
        const uint8_t action_completed = (peer_online &&
                                          completion_valid &&
                                          B2B_Engineer_Feedback.completed_action_seq == mechanism_action_seq) ? 1U : 0U;
        Arm_OneClick_SetMechanismState(peer_ready, action_completed,
                                       (uint8_t)(!peer_online ||
                                           (B2B_Engineer_Feedback.action_bits & DUALBOARD_ACTION_FAULT)));
    }

    if (cmd_sys_state.global_mode == GLOBAL_SAFE_LOCK ||
        cmd_sys_state.global_mode == GLOBAL_MODULE_ERROR ||
        cmd_sys_state.global_mode == GLOBAL_STANDBY) {
        Cmd_Handle_Safe_Mode();
        if (Arm_Control_Debug.oneclick_active) Arm_Control_Config.oneclick_abort = 1U;
        Cmd_Update_Chassis_Debug();
        PubPushMessage(chassis_cmd_pub, &chassis_cmd);
        PubPushMessage(picture_cmd_pub, &picture_cmd);
        Cmd_DualBoard_Sync();
        return;
    }

    Cmd_Update_OneClick_Request();

    if (Cmd_VT13KeyboardActive() || dbus_data.Ctrl_Mode == 1) {
        Cmd_Update_Mouse_Key();
    } else {
        Cmd_Update_Remote_Ctrl();
    }

    Cmd_Update_Chassis_Debug();

    PubPushMessage(chassis_cmd_pub, &chassis_cmd);
    PubPushMessage(picture_cmd_pub, &picture_cmd);

    Cmd_DualBoard_Sync();
}

static void Cmd_Handle_Safe_Mode(void)
{
    chassis_cmd.mode = CHASSIS_CMD_SAFE;

    chassis_cmd.target_vx = 0.0f;
    chassis_cmd.target_vy = 0.0f;
    chassis_cmd.target_vw = 0.0f;
    chassis_cmd.offset_angle = 0.0f;

    picture_cmd.enable = 0U;
    mechanism_test_active = 0U;
    manual_mech_action = DUALBOARD_ACTION_HOLD;
    last_mech_s2 = 0U;
}

// 取通信层交付的原始反馈帧并映射到 B2B_Engineer_Feedback（原 Comm 层 Parse_Engineer_Feedback_Frame 逻辑）。
static void Cmd_Parse_DualBoard_Feedback(void)
{
    uint8_t buf[DUALBOARD_ENGINEER_FEEDBACK_FRAME_LEN];
    uint16_t len = 0;

    if (!DualBoard_Take_Feedback_Frame(buf, sizeof(buf), &len)) return;
    if (len != sizeof(B2B_Engineer_Feedback_Frame_t)) return;

    const B2B_Engineer_Feedback_Frame_t *f = (const B2B_Engineer_Feedback_Frame_t *)buf;
    B2B_Engineer_Feedback.status = (DualBoard_Chassis_Feedback_Status_e)f->status;
    B2B_Engineer_Feedback.chassis_online_bits = f->chassis_online_bits;
    B2B_Engineer_Feedback.mechanism_online_bits = f->mechanism_online_bits;
    B2B_Engineer_Feedback.limit_bits = f->limit_bits;
    B2B_Engineer_Feedback.action_bits = f->action_bits;
    B2B_Engineer_Feedback.error_code = f->error_code;
    B2B_Engineer_Feedback.completed_action_seq = f->completed_action_seq;
    B2B_Engineer_Feedback.picture_lift_pos = f->picture_lift_pos;
    B2B_Engineer_Feedback.picture_transverse_pos = f->picture_transverse_pos;
    B2B_Engineer_Feedback.store_pos_mrad = f->store_pos_mrad;
    B2B_Engineer_Feedback.last_update_ms = HAL_GetTick();
    B2B_Engineer_Feedback.is_online = true;
}

static void Cmd_Update_OneClick_Request(void)
{
    uint8_t keys = 0U;
    uint8_t vt_keys = 0U;
    uint8_t rising;

    if (Cmd_KeyActive(dbus_data.KeyBoard.Shift, vt13_data.KeyBoard.Shift) &&
        Cmd_KeyActive(dbus_data.KeyBoard.Z, vt13_data.KeyBoard.Z)) keys |= 1U << 0;
    if (Cmd_KeyActive(dbus_data.KeyBoard.Shift, vt13_data.KeyBoard.Shift) &&
        Cmd_KeyActive(dbus_data.KeyBoard.X, vt13_data.KeyBoard.X)) keys |= 1U << 1;
    if (Cmd_KeyActive(dbus_data.KeyBoard.Ctrl, vt13_data.KeyBoard.Ctrl) &&
        Cmd_KeyActive(dbus_data.KeyBoard.X, vt13_data.KeyBoard.X)) keys |= 1U << 2;
    if (Cmd_KeyActive(dbus_data.KeyBoard.Ctrl, vt13_data.KeyBoard.Ctrl) &&
        Cmd_KeyActive(dbus_data.KeyBoard.C, vt13_data.KeyBoard.C)) keys |= 1U << 3;
    if (Cmd_KeyActive(dbus_data.KeyBoard.Ctrl, vt13_data.KeyBoard.Ctrl) &&
        Cmd_KeyActive(dbus_data.KeyBoard.V, vt13_data.KeyBoard.V)) keys |= 1U << 4;
    if (Cmd_KeyActive(dbus_data.KeyBoard.Shift, vt13_data.KeyBoard.Shift) &&
        Cmd_KeyActive(dbus_data.KeyBoard.R, vt13_data.KeyBoard.R)) keys |= 1U << 5;
    /* 机械臂挡位下 fn_1/fn_2 用于 J5 正反转，避免同时误触发一键动作。 */
    if (vt13_data.Remote.mode_sw != 2U) {
        if (vt13_data.Remote.fn_1) vt_keys |= 1U << 0;
        if (vt13_data.Remote.fn_2) vt_keys |= 1U << 1;
    }

    rising = (uint8_t)(keys & (uint8_t)~last_action_keys);
    rising |= (uint8_t)((vt_keys & (uint8_t)~last_vt_action_keys) & 0x03U);
    last_action_keys = keys;
    last_vt_action_keys = vt_keys;

    // Ctrl 图传/机构测试模式下，Ctrl+X/C/V 等键复用为图传控制，屏蔽一键请求防冲突。
    if (Cmd_KeyActive(dbus_data.KeyBoard.Ctrl, vt13_data.KeyBoard.Ctrl)) return;

    if (Arm_Control_Debug.oneclick_active) return;
    if (rising & (1U << 0)) Arm_Control_Config.oneclick_request = ARM_ONECLICK_STORE;
    else if (rising & (1U << 1)) Arm_Control_Config.oneclick_request = ARM_ONECLICK_TAKE;
    else if (rising & (1U << 2)) Arm_Control_Config.oneclick_request = ARM_ONECLICK_UNFOLD;
    else if (rising & (1U << 3)) Arm_Control_Config.oneclick_request = ARM_ONECLICK_FOLD;
    else if (rising & (1U << 4)) {
        Arm_Control_Config.oneclick_request =
            (Arm_Control_Debug.oneclick_id == ARM_ONECLICK_SELF_1) ? ARM_ONECLICK_SELF_2 : ARM_ONECLICK_SELF_1;
    }
    else if (rising & (1U << 5)) Arm_Control_Config.oneclick_request = ARM_ONECLICK_RESET;
}

static void Cmd_Update_Remote_Ctrl(void)
{
    picture_cmd.enable = 1U;

    /* VT13 在线时按老车挡位优先接管；DBUS 保留为离线回退，避免两套摇杆叠加。 */
    if (vt13_data.offline.is_online) {
        mechanism_test_active = 0U;
        manual_mech_action = DUALBOARD_ACTION_HOLD;
        last_mech_s2 = 0U;

        if (vt13_data.Remote.mode_sw == 0U) {
            chassis_cmd.target_vx = (float)vt13_data.Remote.Channel[1] * RC_ROCKER_XY_COEF;
            chassis_cmd.target_vy = (float)vt13_data.Remote.Channel[0] * RC_ROCKER_XY_COEF;
            chassis_cmd.target_vw = (float)vt13_data.Remote.Channel[3] * RC_ROCKER_VW_COEF;
            chassis_cmd.mode = CHASSIS_CMD_FREE;
            return;
        }

        if (vt13_data.Remote.mode_sw == 1U) {
            const uint8_t home_trigger = vt13_data.Remote.trigger ? 1U : 0U;
            const int32_t yaw_delta = Stick_Delta(vt13_data.Remote.Channel[0], PICTURE_SERVO_STEP_US);
            const int32_t pitch_delta = Stick_Delta(vt13_data.Remote.Channel[1], PICTURE_SERVO_STEP_US);

            /* 挡位1为纯遥控图传模式：底盘静止，左右摇杆分别控制舵机和两轴机构。 */
            chassis_cmd.mode = CHASSIS_CMD_FREE;
            chassis_cmd.target_vx = 0.0f;
            chassis_cmd.target_vy = 0.0f;
            chassis_cmd.target_vw = 0.0f;
            mechanism_test_active = 1U;

            picture_cmd.yaw_us = Cmd_Limit_Servo_Us((int32_t)picture_cmd.yaw_us + yaw_delta);
            picture_cmd.pitch_us = Cmd_Limit_Servo_Us((int32_t)picture_cmd.pitch_us + pitch_delta);
            Picture_Servo_Set(picture_cmd.yaw_us, picture_cmd.pitch_us);

            picture_cmd.lift = Cmd_Limit_Int32(
                picture_cmd.lift + Stick_Delta(vt13_data.Remote.Channel[2], PICTURE_LIFT_STEP),
                PICTURE_LIFT_MIN, PICTURE_LIFT_MAX);
            picture_cmd.transverse = Cmd_Limit_Int32(
                picture_cmd.transverse - Stick_Delta(vt13_data.Remote.Channel[3], PICTURE_TRANSVERSE_STEP),
                PICTURE_TRANSVERSE_MIN, PICTURE_TRANSVERSE_MAX);

            /* 扳机只在按下边沿发送一次归零，Master1靠action_seq去重。 */
            if (home_trigger && !last_vt_home_trigger) {
                manual_mech_action = DUALBOARD_ACTION_HOME_PICTURE;
                mechanism_action_seq++;
            } else if (!home_trigger) {
                manual_mech_action = DUALBOARD_ACTION_HOLD;
            }
            last_vt_home_trigger = home_trigger;
            return;
        }

        /* 挡位2由 Arm_Ctrl 接管 J1~J6。 */
        chassis_cmd.mode = CHASSIS_CMD_FREE;
        chassis_cmd.target_vx = 0.0f;
        chassis_cmd.target_vy = 0.0f;
        chassis_cmd.target_vw = 0.0f;
        return;
    }

    // 纯 DBUS：S1=1 底盘，S1=2 机械臂，S1=3 图传/丝杠/存矿机构。
    if (dbus_data.Remote.S1 == 2U) {
        // 机械臂模式由 Arm_Ctrl 独占摇杆；底盘与板1机构保持静止。
        chassis_cmd.mode = CHASSIS_CMD_FREE;
        chassis_cmd.target_vx = 0.0f;
        chassis_cmd.target_vy = 0.0f;
        chassis_cmd.target_vw = 0.0f;
        mechanism_test_active = 0U;
        manual_mech_action = DUALBOARD_ACTION_HOLD;
        last_mech_s2 = 0U;
        return;
    }

    if (dbus_data.Remote.S1 == 3U) {
        // 机构模式：底盘冻结(FREE+零速，不触发 board1 STOP_ALL)。
        chassis_cmd.mode = CHASSIS_CMD_FREE;
        chassis_cmd.target_vx = 0.0f;
        chassis_cmd.target_vy = 0.0f;
        chassis_cmd.target_vw = 0.0f;
        Cmd_Update_Stick_Mechanism();
        return;
    }

    // S1=1(上)：底盘模式。
    mechanism_test_active = 0U;
    manual_mech_action = DUALBOARD_ACTION_HOLD;
    last_mech_s2 = 0U;
    chassis_cmd.target_vx = (float)dbus_data.Remote.CH1 * RC_ROCKER_XY_COEF +
                            (float)vt13_data.Remote.Channel[1] * RC_ROCKER_XY_COEF;
    chassis_cmd.target_vy = (float)dbus_data.Remote.CH0 * RC_ROCKER_XY_COEF +
                            (float)vt13_data.Remote.Channel[0] * RC_ROCKER_XY_COEF;
    chassis_cmd.target_vw = (float)dbus_data.Remote.CH2 * RC_ROCKER_VW_COEF +
                            (float)vt13_data.Remote.Channel[3] * RC_ROCKER_VW_COEF;
    chassis_cmd.mode = CHASSIS_CMD_FREE;
}

static void Cmd_Update_Mouse_Key(void)
{
    float mouse_x;
    picture_cmd.enable = 1U;

    if (Cmd_KeyActive(dbus_data.KeyBoard.Ctrl, vt13_data.KeyBoard.Ctrl)) {
        Cmd_Update_Picture_Ctrl();
        Cmd_Update_Mechanism_Test();
        // 底盘保持 FREE 但速度清零：底盘静止，又不触发 board1 图传的 SAFE/STOP_ALL 门控。
        chassis_cmd.mode = CHASSIS_CMD_FREE;
        chassis_cmd.target_vx = 0.0f;
        chassis_cmd.target_vy = 0.0f;
        chassis_cmd.target_vw = 0.0f;
        return;
    }
    mechanism_test_active = 0U;

    chassis_cmd.target_vx = (float)((int32_t)Cmd_KeyActive(dbus_data.KeyBoard.W, vt13_data.KeyBoard.W) -
                                    (int32_t)Cmd_KeyActive(dbus_data.KeyBoard.S, vt13_data.KeyBoard.S)) * KB_WASD_COEF;
    chassis_cmd.target_vy = (float)((int32_t)Cmd_KeyActive(dbus_data.KeyBoard.D, vt13_data.KeyBoard.D) -
                                    (int32_t)Cmd_KeyActive(dbus_data.KeyBoard.A, vt13_data.KeyBoard.A)) * KB_WASD_COEF;

    mouse_x = Cmd_VT13KeyboardActive() ? vt13_data.Mouse.X_Flt : dbus_data.Mouse.X_Flt;
    float active_vw = (float)((int32_t)Cmd_KeyActive(dbus_data.KeyBoard.E, vt13_data.KeyBoard.E) -
                              (int32_t)Cmd_KeyActive(dbus_data.KeyBoard.Q, vt13_data.KeyBoard.Q)) * 3.0f +
                      mouse_x * RC_ROCKER_VW_COEF;

    if (Cmd_KeyActive(dbus_data.KeyBoard.Shift, vt13_data.KeyBoard.Shift)) {
        chassis_cmd.mode = CHASSIS_CMD_SPIN;
        chassis_cmd.target_vw = 5.0f;
    } else {
        chassis_cmd.mode = CHASSIS_CMD_FREE;
        chassis_cmd.target_vw = active_vw;
    }
}

static void Cmd_Update_Picture_Ctrl(void)
{
    int32_t lift_delta = (int32_t)Cmd_KeyActive(dbus_data.KeyBoard.R, vt13_data.KeyBoard.R) -
                         (int32_t)Cmd_KeyActive(dbus_data.KeyBoard.F, vt13_data.KeyBoard.F);
    int32_t transverse_delta = (int32_t)Cmd_KeyActive(dbus_data.KeyBoard.C, vt13_data.KeyBoard.C) -
                               (int32_t)Cmd_KeyActive(dbus_data.KeyBoard.X, vt13_data.KeyBoard.X);
    int32_t yaw_delta = (int32_t)Cmd_KeyActive(dbus_data.KeyBoard.V, vt13_data.KeyBoard.V) -
                        (int32_t)Cmd_KeyActive(dbus_data.KeyBoard.Z, vt13_data.KeyBoard.Z);
    int32_t pitch_delta = (int32_t)Cmd_KeyActive(dbus_data.KeyBoard.G, vt13_data.KeyBoard.G) -
                          (int32_t)Cmd_KeyActive(dbus_data.KeyBoard.B, vt13_data.KeyBoard.B);

    picture_cmd.lift = Cmd_Limit_Int32(picture_cmd.lift + lift_delta * PICTURE_LIFT_STEP,
                                       PICTURE_LIFT_MIN,
                                       PICTURE_LIFT_MAX);
    picture_cmd.transverse = Cmd_Limit_Int32(picture_cmd.transverse + transverse_delta * PICTURE_TRANSVERSE_STEP,
                                             PICTURE_TRANSVERSE_MIN,
                                             PICTURE_TRANSVERSE_MAX);
    picture_cmd.yaw_us = Cmd_Limit_Servo_Us((int32_t)picture_cmd.yaw_us + yaw_delta * PICTURE_SERVO_STEP_US);
    picture_cmd.pitch_us = Cmd_Limit_Servo_Us((int32_t)picture_cmd.pitch_us + pitch_delta * PICTURE_SERVO_STEP_US);

    Picture_Servo_Set(picture_cmd.yaw_us, picture_cmd.pitch_us);
}

// 手动机构测试(不走一键)：Ctrl+Q 归零图传，Ctrl+E 循环槽位存矿。
// 边沿触发 + seq 自增，board1 靠 action_seq 变化去重，一次触发只执行一次。
static void Cmd_Update_Mechanism_Test(void)
{
    uint8_t home_key = Cmd_KeyActive(dbus_data.KeyBoard.Q, vt13_data.KeyBoard.Q);
    uint8_t store_key = Cmd_KeyActive(dbus_data.KeyBoard.E, vt13_data.KeyBoard.E);

    mechanism_test_active = 1U;

    if (home_key && !last_home_key) {
        manual_mech_action = DUALBOARD_ACTION_HOME_PICTURE;
        mechanism_action_seq++;
    } else if (store_key && !last_store_key) {
        manual_store_slot = (uint8_t)((manual_store_slot + 1U) & 0x03U);  // 0→1→2→3→0
        manual_mech_action = DUALBOARD_ACTION_EXECUTE;
        mechanism_action_seq++;
    }

    last_home_key = home_key;
    last_store_key = store_key;
}

// 摇杆偏移(去中点后 ±660)带死区换算成每周期步进。
static int32_t Stick_Delta(int16_t ch, int32_t step)
{
    int32_t v = ch;
    if (v > -RC_MECH_DEADZONE && v < RC_MECH_DEADZONE) return 0;
    if (v > 0) v -= RC_MECH_DEADZONE; else v += RC_MECH_DEADZONE;
    return v * step / RC_STICK_RANGE;
}

// 纯 DBUS 机构模式(S1=3)：
//  S2中(3)：左摇杆上下/左右调图传升降和横移；
//  S2上(1)：边沿触发图传+丝杠归零；S2下(2)：边沿触发存矿，拨轮选择槽位0~3。
static void Cmd_Update_Stick_Mechanism(void)
{
    const uint8_t s2 = dbus_data.Remote.S2;
    /* 0 是刚进入机构模式的未武装哨兵，避免带着上/下挡进入时立即误触发动作。 */
    const uint8_t s2_changed =
        (last_mech_s2 != 0U && s2 != last_mech_s2) ? 1U : 0U;
    int32_t slot = (dbus_data.Remote.Dial + 660) * 4 / 1320;

    mechanism_test_active = 1U;
    manual_mech_action = DUALBOARD_ACTION_HOLD;

    if (slot < 0) slot = 0;
    if (slot > 3) slot = 3;
    manual_store_slot = (uint8_t)slot;

    if (s2 == 3U) {
        // 图传：CH2=左摇杆上下(升降)，CH3=左摇杆左右(横移)。
        picture_cmd.lift = Cmd_Limit_Int32(picture_cmd.lift + Stick_Delta(dbus_data.Remote.CH2, PICTURE_LIFT_STEP),
                                           PICTURE_LIFT_MIN, PICTURE_LIFT_MAX);
        picture_cmd.transverse = Cmd_Limit_Int32(picture_cmd.transverse - Stick_Delta(dbus_data.Remote.CH3, PICTURE_TRANSVERSE_STEP),
                                                 PICTURE_TRANSVERSE_MIN, PICTURE_TRANSVERSE_MAX);
    } else if (s2 == 1U && s2_changed) {
        manual_mech_action = DUALBOARD_ACTION_HOME_PICTURE;
        mechanism_action_seq++;
    } else if (s2 == 2U && s2_changed) {
        manual_mech_action = DUALBOARD_ACTION_EXECUTE;
        mechanism_action_seq++;
    }
    last_mech_s2 = s2;
}

static void Cmd_Update_Chassis_Debug(void)
{
    Chassis_Debug_RemoteOnline = (vt13_data.offline.is_online || dbus_data.offline.is_online) ? 1U : 0U;

    Chassis_Debug_Readback.mode = chassis_cmd.mode;
    Chassis_Debug_Readback.target_vx = chassis_cmd.target_vx;
    Chassis_Debug_Readback.target_vy = chassis_cmd.target_vy;
    Chassis_Debug_Readback.target_vw = chassis_cmd.target_vw;
    Chassis_Debug_Readback.offset_angle = chassis_cmd.offset_angle;
}

static void Cmd_DualBoard_Sync(void)
{
    static uint8_t remote_tx_divider = 0U;
    const Arm_OneClick_Output_t *action = Arm_OneClick_GetOutput();
    int32_t lift = picture_cmd.lift;
    int32_t transverse = picture_cmd.transverse;
    uint8_t store_slot = Arm_Control_Config.oneclick_store_slot;
    DualBoard_Mechanism_Action_e mechanism_action =
        picture_cmd.enable ? DUALBOARD_ACTION_HOLD : DUALBOARD_ACTION_STOP_ALL;
    uint8_t action_override = 0U;

    if (chassis_cmd.mode == CHASSIS_CMD_SAFE) {
        mechanism_action = DUALBOARD_ACTION_STOP_ALL;
    } else if (action != NULL && action->active && action->picture_override) {
        lift = action->picture_lift;
        transverse = action->picture_transverse;
        store_slot = action->store_slot;
        mechanism_action = (DualBoard_Mechanism_Action_e)action->mechanism_action;
        action_override = 1U;
        Picture_Servo_Set(action->yaw_us, action->pitch_us);
    } else if (mechanism_test_active) {
        // 手动机构测试：图传升降/横移目标已在 picture_cmd 里，动作用手动锁存值。
        // seq 已在 Cmd_Update_Mechanism_Test 内按边沿自增，此处不再动。
        mechanism_action = manual_mech_action;
        store_slot = manual_store_slot;
    } else if (action != NULL && action->mechanism_action == DUALBOARD_ACTION_STOP_ALL) {
        mechanism_action = DUALBOARD_ACTION_STOP_ALL;
    }

    if (action_override && !oneclick_override_last) mechanism_action_seq++;
    oneclick_override_last = action_override;

    // Robot_Cmd 为 200Hz；USART10 的 73B 固定帧降为 100Hz，单帧约 6.4ms 可在下一帧前发完。
    remote_tx_divider++;
    if (remote_tx_divider < 2U) return;
    remote_tx_divider = 0U;

    uint8_t dbus_raw[18];
    uint8_t vt13_raw[21];
    float arm_position[6];
    uint8_t arm_online_mask;
    uint8_t remote_online_bits = 0U;
    B2B_Aux_Command_t auxiliary = {
        .global_mode = (uint8_t)cmd_sys_state.global_mode,
        .mechanism_action = (uint8_t)mechanism_action,
        .store_slot = store_slot,
        .action_seq = mechanism_action_seq,
        .picture_lift = lift,
        .picture_transverse = transverse,
        .ui_flags = Arm_Control_Debug.clamp_close ? DUALBOARD_UI_CLAMP_CLOSED : 0U,
        .aux_flags = 0U,
    };

    if (dbus_data.offline.is_online) remote_online_bits |= DUALBOARD_REMOTE_DBUS_ONLINE;
    if (vt13_data.offline.is_online && vt13_data.CRC_flag) remote_online_bits |= DUALBOARD_REMOTE_VT13_ONLINE;
    if (action_override || mechanism_action == DUALBOARD_ACTION_STOP_ALL) {
        auxiliary.aux_flags |= DUALBOARD_AUX_OVERRIDE_ACTIVE;
    }

    // 两个快照都在各自完整帧解析回调里更新；关中断复制，确保不会取到半新半旧的数据。
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memcpy(dbus_raw, DBUS_RAW_SNAPSHOT, sizeof(dbus_raw));
    memcpy(vt13_raw, VT13_RAW_SNAPSHOT, sizeof(vt13_raw));
    for (uint8_t axis = 0U; axis < 6U; ++axis) {
        arm_position[axis] = Arm_Control_Debug.position[axis];
    }
    arm_online_mask = (uint8_t)(Arm_Control_Debug.online_mask & 0x3FU);
    if (primask == 0U) __enable_irq();

    uint8_t send_result = DualBoard_Send_Remote(&huart10, dbus_raw, vt13_raw,
                                                 remote_online_bits, &auxiliary,
                                                 arm_position, arm_online_mask);
    /* 调试快照体积较大，放入静态存储区，避免占用 Command 任务栈。 */
    static B2B_Board2_Debug_t debug_snapshot = {0};
    debug_snapshot.last_update_ms = HAL_GetTick();
    debug_snapshot.remote_online_bits = remote_online_bits;
    debug_snapshot.last_send_result = send_result;
    debug_snapshot.dbus = dbus_data;
    debug_snapshot.vt13 = vt13_data;
    debug_snapshot.chassis_cmd = chassis_cmd;
    debug_snapshot.picture_cmd = picture_cmd;
    debug_snapshot.auxiliary = auxiliary;
    debug_snapshot.board1_feedback = B2B_Engineer_Feedback;
    B2B_Board2_Debug = debug_snapshot;
}

static int32_t Cmd_Limit_Int32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static uint16_t Cmd_Limit_Servo_Us(int32_t value)
{
    return Picture_Servo_Clamp_Us(value);
}

static uint8_t KeyActive(uint8_t key_state)
{
    return key_state ? 1U : 0U;
}

static uint8_t Cmd_KeyActive(uint8_t dbus_key, uint8_t vt13_key)
{
    return (uint8_t)(KeyActive(dbus_key) ||
                     (vt13_data.offline.is_online && KeyActive(vt13_key)));
}

static uint8_t Cmd_VT13KeyboardActive(void)
{
    return (uint8_t)(vt13_data.offline.is_online && vt13_data.Ctrl_Mode == 1);
}
