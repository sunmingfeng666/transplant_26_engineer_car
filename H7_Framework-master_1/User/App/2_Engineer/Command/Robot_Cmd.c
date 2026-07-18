//
// 工程车底盘/机构执行板命令中心。
// USART10 接收机械臂板转发的 DBUS、VT13 原始帧；本板完成遥控解算并生成执行命令。
//
#include "Robot_Cmd.h"
#include "Comm_DualBoard.h"
#include "Message_Center.h"
#include "System_State.h"
#include "DBUS.h"
#include "VT13.h"
#include "LeadScrew_Ctrl.h"
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
// 丝杠行程需与 LeadScrew_Ctrl.c 的 LEADSCREW_TRAVEL_MIN/MAX 一致；步进可调。
#define LEADSCREW_TARGET_MIN 0
#define LEADSCREW_TARGET_MAX 900000
#define LEADSCREW_STEP 4000
#define RC_MECH_DEADZONE 40
#define RC_STICK_RANGE 620

static Subscriber_t *sys_state_sub;
static Publisher_t *chassis_cmd_pub;

static System_State_t cmd_sys_state;
static Chassis_Cmd_t chassis_cmd = {0};

/*
 *
 * 该结构不参与控制，只在收到并解析完一帧有效V3数据后整体更新。
 */
typedef struct {
    uint8_t last_seq;
    uint8_t online_bits;
    uint32_t last_update_ms;
    uint8_t dbus_raw[18];
    uint8_t vt13_raw[21];
    DBUS_Typedef dbus;
    VT13_Typedef vt13;
    B2B_Aux_Command_t auxiliary;
    B2B_Arm_Feedback_t arm_feedback;
} B2B_Remote_Debug_t;

volatile B2B_Remote_Debug_t B2B_Remote_Debug = {0};

static B2B_Aux_Command_t remote_auxiliary = {0};
static uint32_t remote_last_update_ms;
static uint8_t remote_last_seq;
static uint8_t remote_online_bits;
static uint8_t auxiliary_override_last;
static int32_t manual_picture_lift;
static int32_t manual_picture_transverse;
static int32_t manual_leadscrew;

static void Cmd_Handle_Safe_Mode(void);
static void Cmd_Parse_DualBoard_Frames(void);
static void Cmd_Update_From_Remote(void);
static void Cmd_Update_Manual_Picture(void);
static void Cmd_Update_Manual_LeadScrew(void);
static void Cmd_Apply_Output(DualBoard_Chassis_Mode_e mode, float vx_m_s, float vy_m_s, float vw_rad_s);
static uint8_t Cmd_KeyActive(uint8_t dbus_key, uint8_t vt13_key);
static uint8_t Cmd_VT13KeyboardActive(void);
static int32_t Cmd_Stick_Delta(int16_t ch, int32_t step);
static int32_t Cmd_Limit_Int32(int32_t value, int32_t min_value, int32_t max_value);

void Robot_Cmd_Init(void)
{
    sys_state_sub = SubRegister("system_state", sizeof(System_State_t));
    chassis_cmd_pub = PubRegister("chassis_cmd", &chassis_cmd, sizeof(Chassis_Cmd_t));

    remote_auxiliary.mechanism_action = DUALBOARD_ACTION_STOP_ALL;
    manual_picture_lift = 0;
    manual_picture_transverse = 0;
    manual_leadscrew = LEADSCREW_TARGET_MIN;
}

void Robot_Cmd_Update(void)
{
    if (sys_state_sub) SubGetMessage(sys_state_sub, &cmd_sys_state);

    Cmd_Parse_DualBoard_Frames();
    Cmd_Update_From_Remote();

    // 本地主题只维持框架接口；真实执行命令写入 B2B_Chassis_Cmd / B2B_Picture_Cmd。
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

// 取通信层已通过 SOF/版本/CRC16/TAIL 校验的 V3 帧，并解算两份遥控原始数据。
static void Cmd_Parse_DualBoard_Frames(void)
{
    uint8_t buf[DUALBOARD_REMOTE_FRAME_LEN];
    uint16_t len = 0U;
    // 调试快照体积较大，使用静态存储，避免耗尽仅 512 字节的 Command 任务栈。
    static B2B_Remote_Debug_t debug_snapshot = {0};

    if (!DualBoard_Take_Cmd_Frame(buf, sizeof(buf), &len)) return;
    if (len != sizeof(B2B_Remote_Frame_t)) return;

    const B2B_Remote_Frame_t *frame = (const B2B_Remote_Frame_t *)buf;
    if (frame->auxiliary.global_mode > GLOBAL_NORMAL_MATCH) return;
    if (frame->auxiliary.mechanism_action > DUALBOARD_ACTION_CLEAR_FAULT) return;
    if (frame->auxiliary.store_slot > 3U) return;
    if (frame->remote_online_bits & (uint8_t)~(DUALBOARD_REMOTE_DBUS_ONLINE | DUALBOARD_REMOTE_VT13_ONLINE)) return;
    if (frame->arm_online_mask & (uint8_t)~0x3FU) return;

    remote_online_bits = frame->remote_online_bits;
    remote_auxiliary = frame->auxiliary;
    remote_last_seq = frame->seq;
    remote_last_update_ms = HAL_GetTick();

    B2B_Arm_Feedback.online_mask = frame->arm_online_mask;
    B2B_Arm_Feedback.last_update_ms = remote_last_update_ms;
    B2B_Arm_Feedback.is_online = true;
    for (uint8_t axis = 0U; axis < 6U; ++axis) {
        B2B_Arm_Feedback.position[axis] = (float)frame->arm_position_mrad[axis] * 0.001f;
    }

    if (remote_online_bits & DUALBOARD_REMOTE_DBUS_ONLINE) {
        DBUS_Resolved((uint8_t *)frame->dbus_raw, &DBUS, sizeof(frame->dbus_raw));
        DBUS.offline.is_online = true;
    } else {
        DBUS.offline.is_online = false;
    }

    if (remote_online_bits & DUALBOARD_REMOTE_VT13_ONLINE) {
        VT13_Resolved((uint8_t *)frame->vt13_raw, &VT13, sizeof(frame->vt13_raw));
        VT13.offline.is_online = VT13.CRC_flag;
    } else {
        VT13.offline.is_online = false;
    }

    // 所有业务字段解析完成后再整体写入调试镜像，避免观察到半更新状态。
    debug_snapshot.last_seq = remote_last_seq;
    debug_snapshot.online_bits = remote_online_bits;
    debug_snapshot.last_update_ms = remote_last_update_ms;
    memcpy(debug_snapshot.dbus_raw, frame->dbus_raw, sizeof(debug_snapshot.dbus_raw));
    memcpy(debug_snapshot.vt13_raw, frame->vt13_raw, sizeof(debug_snapshot.vt13_raw));
    debug_snapshot.dbus = DBUS;
    debug_snapshot.vt13 = VT13;
    debug_snapshot.auxiliary = remote_auxiliary;
    debug_snapshot.arm_feedback = B2B_Arm_Feedback;
    B2B_Remote_Debug = debug_snapshot;
}

static void Cmd_Update_From_Remote(void)
{
    const uint32_t now = HAL_GetTick();
    const uint8_t link_online = (remote_last_update_ms != 0U &&
                                 (now - remote_last_update_ms) <= DUALBOARD_CHASSIS_TIMEOUT_MS) ? 1U : 0U;
    B2B_Arm_Feedback.is_online = link_online ? true : false;
    const uint8_t remote_online = (DBUS.offline.is_online || VT13.offline.is_online) ? 1U : 0U;
    const uint8_t auxiliary_override =
        (remote_auxiliary.aux_flags & DUALBOARD_AUX_OVERRIDE_ACTIVE) ? 1U : 0U;

    System_State_Report_Remote((uint8_t)(link_online && remote_online));

    if (!link_online || !remote_online || remote_auxiliary.global_mode != GLOBAL_NORMAL_MATCH) {
        Cmd_Apply_Output(DUALBOARD_CHASSIS_SAFE, 0.0f, 0.0f, 0.0f);
        B2B_Picture_Cmd.picture_lift = manual_picture_lift;
        B2B_Picture_Cmd.picture_transverse = manual_picture_transverse;
        B2B_Picture_Cmd.mechanism_action = DUALBOARD_ACTION_STOP_ALL;
        B2B_Picture_Cmd.store_slot = remote_auxiliary.store_slot;
        B2B_Picture_Cmd.action_seq = remote_auxiliary.action_seq;
        B2B_Picture_Cmd.ui_flags = remote_auxiliary.ui_flags;
        B2B_Picture_Cmd.last_seq = remote_last_seq;
        B2B_Picture_Cmd.last_update_ms = remote_last_update_ms;
        B2B_Picture_Cmd.is_online = link_online ? true : false;
        auxiliary_override_last = auxiliary_override;
        return;
    }

    if (Cmd_VT13KeyboardActive() || DBUS.Ctrl_Mode == 1U) {
        if (Cmd_KeyActive(DBUS.KeyBoard.Ctrl, VT13.KeyBoard.Ctrl)) {
            Cmd_Apply_Output(DUALBOARD_CHASSIS_FREE, 0.0f, 0.0f, 0.0f);
        } else {
            const float vx = (float)((int32_t)Cmd_KeyActive(DBUS.KeyBoard.W, VT13.KeyBoard.W) -
                                     (int32_t)Cmd_KeyActive(DBUS.KeyBoard.S, VT13.KeyBoard.S)) * KB_WASD_COEF;
            const float vy = (float)((int32_t)Cmd_KeyActive(DBUS.KeyBoard.D, VT13.KeyBoard.D) -
                                     (int32_t)Cmd_KeyActive(DBUS.KeyBoard.A, VT13.KeyBoard.A)) * KB_WASD_COEF;
            const float mouse_x = Cmd_VT13KeyboardActive() ? VT13.Mouse.X_Flt : DBUS.Mouse.X_Flt;
            const float vw = (float)((int32_t)Cmd_KeyActive(DBUS.KeyBoard.E, VT13.KeyBoard.E) -
                                     (int32_t)Cmd_KeyActive(DBUS.KeyBoard.Q, VT13.KeyBoard.Q)) * 3.0f +
                             mouse_x * RC_ROCKER_VW_COEF;
            if (Cmd_KeyActive(DBUS.KeyBoard.Shift, VT13.KeyBoard.Shift)) {
                Cmd_Apply_Output(DUALBOARD_CHASSIS_SPIN, vx, vy, 5.0f);
            } else {
                Cmd_Apply_Output(DUALBOARD_CHASSIS_FREE, vx, vy, vw);
            }
        }
    } else if (VT13.offline.is_online) {
        if (VT13.Remote.mode_sw == 0U) {
            Cmd_Apply_Output(DUALBOARD_CHASSIS_FREE,
                             (float)VT13.Remote.Channel[1] * RC_ROCKER_XY_COEF,
                             (float)VT13.Remote.Channel[0] * RC_ROCKER_XY_COEF,
                             (float)VT13.Remote.Channel[3] * RC_ROCKER_VW_COEF);
        } else {
            Cmd_Apply_Output(DUALBOARD_CHASSIS_FREE, 0.0f, 0.0f, 0.0f);
        }
    } else if (DBUS.Remote.S1 == 1U) {
        // 纯 DBUS 底盘模式：S2 下挡进入小陀螺；上/中挡保持普通底盘控制。
        // 小陀螺仍允许 CH0/CH1 平移，CH2 暂不参与旋转，避免与固定角速度冲突。
        const float vx = (float)DBUS.Remote.CH1 * RC_ROCKER_XY_COEF;
        const float vy = (float)DBUS.Remote.CH0 * RC_ROCKER_XY_COEF;
        if (DBUS.Remote.S2 == 2U) {
            Cmd_Apply_Output(DUALBOARD_CHASSIS_SPIN, vx, vy, 5.0f);
        } else {
            Cmd_Apply_Output(DUALBOARD_CHASSIS_FREE,
                             vx,
                             vy,
                             (float)DBUS.Remote.CH2 * RC_ROCKER_VW_COEF);
        }
    } else {
        Cmd_Apply_Output(DUALBOARD_CHASSIS_FREE, 0.0f, 0.0f, 0.0f);
    }

    if (!auxiliary_override) Cmd_Update_Manual_Picture();
    Cmd_Update_Manual_LeadScrew();
    if (!auxiliary_override && auxiliary_override_last) {
        // 一键动作刚退出时，以机械臂板当前手动目标重新对齐，避免目标突跳。
        manual_picture_lift = remote_auxiliary.picture_lift;
        manual_picture_transverse = remote_auxiliary.picture_transverse;
    }

    B2B_Picture_Cmd.picture_lift = auxiliary_override ? remote_auxiliary.picture_lift : manual_picture_lift;
    B2B_Picture_Cmd.picture_transverse = auxiliary_override ? remote_auxiliary.picture_transverse : manual_picture_transverse;
    B2B_Picture_Cmd.mechanism_action = (DualBoard_Mechanism_Action_e)remote_auxiliary.mechanism_action;
    B2B_Picture_Cmd.store_slot = remote_auxiliary.store_slot;
    B2B_Picture_Cmd.action_seq = remote_auxiliary.action_seq;
    B2B_Picture_Cmd.ui_flags = remote_auxiliary.ui_flags;
    B2B_Picture_Cmd.last_seq = remote_last_seq;
    B2B_Picture_Cmd.last_update_ms = remote_last_update_ms;
    B2B_Picture_Cmd.is_online = true;
    auxiliary_override_last = auxiliary_override;
}

// 手动图传目标由本板依据原始遥控帧生成；一键覆盖期间本函数不运行。
static void Cmd_Update_Manual_Picture(void)
{
    if (Cmd_KeyActive(DBUS.KeyBoard.Ctrl, VT13.KeyBoard.Ctrl) &&
        (Cmd_VT13KeyboardActive() || DBUS.Ctrl_Mode == 1U)) {
        const int32_t lift_delta = (int32_t)Cmd_KeyActive(DBUS.KeyBoard.R, VT13.KeyBoard.R) -
                                   (int32_t)Cmd_KeyActive(DBUS.KeyBoard.F, VT13.KeyBoard.F);
        const int32_t transverse_delta = (int32_t)Cmd_KeyActive(DBUS.KeyBoard.C, VT13.KeyBoard.C) -
                                         (int32_t)Cmd_KeyActive(DBUS.KeyBoard.X, VT13.KeyBoard.X);
        manual_picture_lift = Cmd_Limit_Int32(manual_picture_lift + lift_delta * PICTURE_LIFT_STEP,
                                              PICTURE_LIFT_MIN, PICTURE_LIFT_MAX);
        manual_picture_transverse = Cmd_Limit_Int32(
            manual_picture_transverse + transverse_delta * PICTURE_TRANSVERSE_STEP,
            PICTURE_TRANSVERSE_MIN, PICTURE_TRANSVERSE_MAX);
        return;
    }

    if (VT13.offline.is_online && VT13.Remote.mode_sw == 1U) {
        manual_picture_lift = Cmd_Limit_Int32(
            manual_picture_lift + Cmd_Stick_Delta(VT13.Remote.Channel[2], PICTURE_LIFT_STEP),
            PICTURE_LIFT_MIN, PICTURE_LIFT_MAX);
        manual_picture_transverse = Cmd_Limit_Int32(
            manual_picture_transverse - Cmd_Stick_Delta(VT13.Remote.Channel[3], PICTURE_TRANSVERSE_STEP),
            PICTURE_TRANSVERSE_MIN, PICTURE_TRANSVERSE_MAX);
        return;
    }

    if (!VT13.offline.is_online && DBUS.Remote.S1 == 3U) {
        manual_picture_lift = Cmd_Limit_Int32(
            manual_picture_lift + Cmd_Stick_Delta(DBUS.Remote.CH2, PICTURE_LIFT_STEP),
            PICTURE_LIFT_MIN, PICTURE_LIFT_MAX);
        manual_picture_transverse = Cmd_Limit_Int32(
            manual_picture_transverse - Cmd_Stick_Delta(DBUS.Remote.CH3, PICTURE_TRANSVERSE_STEP),
            PICTURE_TRANSVERSE_MIN, PICTURE_TRANSVERSE_MAX);
    }
}

// 手动丝杠目标：键盘 Ctrl+G/Z，或机构模式 S1=3、S2=3 时用右摇杆 CH1 上下调节。
// 仅在丝杠回零完成并进入位置跟踪状态后接受摇杆，避免回零后目标突跳。
static void Cmd_Update_Manual_LeadScrew(void)
{
    const Engineer_LeadScrew_Status_t status = Engineer_LeadScrew_Get_Status();

    if (!status.homing_done || status.state != ENGINEER_LEADSCREW_TRACKING) {
        manual_leadscrew = LEADSCREW_TARGET_MIN;
        Engineer_LeadScrew_Set_Target(manual_leadscrew);
        return;
    }

    if (Cmd_KeyActive(DBUS.KeyBoard.Ctrl, VT13.KeyBoard.Ctrl) &&
        (Cmd_VT13KeyboardActive() || DBUS.Ctrl_Mode == 1U)) {
        const int32_t delta = (int32_t)Cmd_KeyActive(DBUS.KeyBoard.G, VT13.KeyBoard.G) -
                              (int32_t)Cmd_KeyActive(DBUS.KeyBoard.Z, VT13.KeyBoard.Z);
        manual_leadscrew = Cmd_Limit_Int32(manual_leadscrew + delta * LEADSCREW_STEP,
                                           LEADSCREW_TARGET_MIN, LEADSCREW_TARGET_MAX);
    } else if (!VT13.offline.is_online &&
               DBUS.offline.is_online &&
               DBUS.Remote.S1 == 3U &&
               DBUS.Remote.S2 == 3U) {
        manual_leadscrew = Cmd_Limit_Int32(
            manual_leadscrew + Cmd_Stick_Delta(DBUS.Remote.CH1, LEADSCREW_STEP),
            LEADSCREW_TARGET_MIN, LEADSCREW_TARGET_MAX);
    }
    Engineer_LeadScrew_Set_Target(manual_leadscrew);
}

static void Cmd_Apply_Output(DualBoard_Chassis_Mode_e mode, float vx_m_s, float vy_m_s, float vw_rad_s)
{
    B2B_Chassis_Cmd.mode = mode;
    B2B_Chassis_Cmd.vx_mm_s = vx_m_s * 1000.0f;
    B2B_Chassis_Cmd.vy_mm_s = vy_m_s * 1000.0f;
    B2B_Chassis_Cmd.vw_mrad_s = vw_rad_s * 1000.0f;
    B2B_Chassis_Cmd.last_seq = remote_last_seq;
    B2B_Chassis_Cmd.last_update_ms = remote_last_update_ms;
    B2B_Chassis_Cmd.is_online =
        (remote_last_update_ms != 0U &&
         (HAL_GetTick() - remote_last_update_ms) <= DUALBOARD_CHASSIS_TIMEOUT_MS) ? true : false;
}

static uint8_t Cmd_KeyActive(uint8_t dbus_key, uint8_t vt13_key)
{
    return (uint8_t)((dbus_key != 0U) || (VT13.offline.is_online && vt13_key != 0U));
}

static uint8_t Cmd_VT13KeyboardActive(void)
{
    return (uint8_t)(VT13.offline.is_online && VT13.Ctrl_Mode == 1U);
}

static int32_t Cmd_Stick_Delta(int16_t ch, int32_t step)
{
    int32_t value = ch;
    if (value > -RC_MECH_DEADZONE && value < RC_MECH_DEADZONE) return 0;
    if (value > 0) value -= RC_MECH_DEADZONE;
    else value += RC_MECH_DEADZONE;
    return value * step / RC_STICK_RANGE;
}

static int32_t Cmd_Limit_Int32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}
