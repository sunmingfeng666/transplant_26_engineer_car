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

static void Cmd_Handle_Safe_Mode(void);
static void Cmd_Parse_DualBoard_Feedback(void);
static void Cmd_Update_Remote_Ctrl(void);
static void Cmd_Update_Mouse_Key(void);
static void Cmd_Update_Picture_Ctrl(void);
static void Cmd_Update_OneClick_Request(void);
static void Cmd_Update_Chassis_Debug(void);
static void Cmd_DualBoard_Sync(void);
static DualBoard_Chassis_Mode_e Cmd_To_DualBoard_Mode(Chassis_Mode_e mode);
static int32_t Cmd_Limit_Int32(int32_t value, int32_t min_value, int32_t max_value);
static uint16_t Cmd_Limit_Servo_Us(int32_t value);
static uint8_t KeyActive(uint8_t key_state);

static uint8_t mechanism_action_seq;
static uint8_t last_action_keys;
static uint8_t last_vt_action_keys;
static uint8_t oneclick_override_last;

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

    if (dbus_data.Ctrl_Mode == 1) {
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

    if (KeyActive(dbus_data.KeyBoard.Shift) && KeyActive(dbus_data.KeyBoard.Z)) keys |= 1U << 0;
    if (KeyActive(dbus_data.KeyBoard.Shift) && KeyActive(dbus_data.KeyBoard.X)) keys |= 1U << 1;
    if (KeyActive(dbus_data.KeyBoard.Ctrl) && KeyActive(dbus_data.KeyBoard.X)) keys |= 1U << 2;
    if (KeyActive(dbus_data.KeyBoard.Ctrl) && KeyActive(dbus_data.KeyBoard.C)) keys |= 1U << 3;
    if (KeyActive(dbus_data.KeyBoard.Ctrl) && KeyActive(dbus_data.KeyBoard.V)) keys |= 1U << 4;
    if (KeyActive(dbus_data.KeyBoard.Shift) && KeyActive(dbus_data.KeyBoard.R)) keys |= 1U << 5;
    if (vt13_data.Remote.fn_1) vt_keys |= 1U << 0;
    if (vt13_data.Remote.fn_2) vt_keys |= 1U << 1;

    rising = (uint8_t)(keys & (uint8_t)~last_action_keys);
    rising |= (uint8_t)((vt_keys & (uint8_t)~last_vt_action_keys) & 0x03U);
    last_action_keys = keys;
    last_vt_action_keys = vt_keys;

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
    picture_cmd.enable = 1U;

    if (KeyActive(dbus_data.KeyBoard.Ctrl)) {
        Cmd_Update_Picture_Ctrl();
        chassis_cmd.mode = CHASSIS_CMD_SAFE;
        chassis_cmd.target_vx = 0.0f;
        chassis_cmd.target_vy = 0.0f;
        chassis_cmd.target_vw = 0.0f;
        return;
    }

    chassis_cmd.target_vx = (float)(dbus_data.KeyBoard.W - dbus_data.KeyBoard.S) * KB_WASD_COEF;
    chassis_cmd.target_vy = (float)(dbus_data.KeyBoard.D - dbus_data.KeyBoard.A) * KB_WASD_COEF;

    float active_vw = (float)(dbus_data.KeyBoard.E - dbus_data.KeyBoard.Q) * 3.0f +
                      dbus_data.Mouse.X_Flt * RC_ROCKER_VW_COEF;

    if (KeyActive(dbus_data.KeyBoard.Shift)) {
        chassis_cmd.mode = CHASSIS_CMD_SPIN;
        chassis_cmd.target_vw = 5.0f;
    } else {
        chassis_cmd.mode = CHASSIS_CMD_FREE;
        chassis_cmd.target_vw = active_vw;
    }
}

static void Cmd_Update_Picture_Ctrl(void)
{
    int32_t lift_delta = (int32_t)KeyActive(dbus_data.KeyBoard.R) -
                         (int32_t)KeyActive(dbus_data.KeyBoard.F);
    int32_t transverse_delta = (int32_t)KeyActive(dbus_data.KeyBoard.C) -
                               (int32_t)KeyActive(dbus_data.KeyBoard.X);
    int32_t yaw_delta = (int32_t)KeyActive(dbus_data.KeyBoard.V) -
                        (int32_t)KeyActive(dbus_data.KeyBoard.Z);
    int32_t pitch_delta = (int32_t)KeyActive(dbus_data.KeyBoard.G) -
                          (int32_t)KeyActive(dbus_data.KeyBoard.B);

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
    } else if (action != NULL && action->mechanism_action == DUALBOARD_ACTION_STOP_ALL) {
        mechanism_action = DUALBOARD_ACTION_STOP_ALL;
    }

    if (action_override && !oneclick_override_last) mechanism_action_seq++;
    oneclick_override_last = action_override;

    DualBoard_Send_Engineer(&huart10,
                            Cmd_To_DualBoard_Mode(chassis_cmd.mode),
                            chassis_cmd.target_vx * 1000.0f,
                            chassis_cmd.target_vy * 1000.0f,
                            chassis_cmd.target_vw * 1000.0f,
                            lift,
                            transverse,
                            mechanism_action,
                            store_slot,
                            mechanism_action_seq,
                            Arm_Control_Debug.clamp_close ? DUALBOARD_UI_CLAMP_CLOSED : 0U);
}

static DualBoard_Chassis_Mode_e Cmd_To_DualBoard_Mode(Chassis_Mode_e mode)
{
    if (mode == CHASSIS_CMD_SAFE) return DUALBOARD_CHASSIS_SAFE;
    if (mode == CHASSIS_CMD_SPIN) return DUALBOARD_CHASSIS_SPIN;
    return DUALBOARD_CHASSIS_FREE;
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
