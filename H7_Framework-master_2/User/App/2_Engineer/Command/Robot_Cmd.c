//
// 工程车遥控接收/发送板命令中心。
// 数据路径：
// DBUS/VT13 -> chassis_cmd topic -> USART10 发送底盘命令帧。
// 底盘板会通过同一个 USART10 回传状态反馈帧。
//
#include "Robot_Cmd.h"

#include "Message_Center.h"
#include "System_State.h"
#include "DBUS.h"
#include "VT13.h"
#include "Comm_DualBoard.h"
#include "usart.h"

#define RC_ROCKER_XY_COEF      0.004f
#define RC_ROCKER_VW_COEF      0.02f
#define KB_WASD_COEF           1.0f

// 订阅器读取系统和设备数据，发布器维持框架内部 App 主题流。
static Subscriber_t *sys_state_sub;
static Subscriber_t *dbus_sub;
static Subscriber_t *vt13_sub;

static Publisher_t *chassis_cmd_pub;
static Publisher_t *gimbal_cmd_pub;
static Publisher_t *shoot_cmd_pub;

static System_State_t cmd_sys_state;
static DBUS_Typedef dbus_data;
static VT13_Typedef vt13_data;

static Chassis_Cmd_t chassis_cmd = {0};
static Gimbal_Cmd_t gimbal_cmd = {0};
static Shoot_Cmd_t shoot_cmd = {0};

volatile Chassis_Cmd_t Chassis_Debug_Readback __attribute__((used)) = {0};
volatile uint8_t Chassis_Debug_RemoteOnline __attribute__((used)) = 0U;

static void Cmd_Handle_Safe_Mode(void);
static void Cmd_Update_Remote_Ctrl(void);
static void Cmd_Update_Mouse_Key(void);
static void Cmd_Update_Chassis_Debug(void);
static void Cmd_DualBoard_Sync(void);
static DualBoard_Chassis_Mode_e Cmd_To_DualBoard_Mode(Chassis_Mode_e mode);

void Robot_Cmd_Init(void)
{
    sys_state_sub = SubRegister("system_state", sizeof(System_State_t));
    dbus_sub      = SubRegister("dbus_data", sizeof(DBUS_Typedef));
    vt13_sub      = SubRegister("vt13_data", sizeof(VT13_Typedef));

    chassis_cmd_pub = PubRegister("chassis_cmd", &chassis_cmd, sizeof(Chassis_Cmd_t));
    gimbal_cmd_pub  = PubRegister("gimbal_cmd", &gimbal_cmd, sizeof(Gimbal_Cmd_t));
    shoot_cmd_pub   = PubRegister("shoot_cmd", &shoot_cmd, sizeof(Shoot_Cmd_t));
}

void Robot_Cmd_Update(void)
{
    // Message_Center 会取出设备任务/中断回调发布的最新数据。
    if (sys_state_sub) SubGetMessage(sys_state_sub, &cmd_sys_state);
    if (dbus_sub) SubGetMessage(dbus_sub, &dbus_data);
    if (vt13_sub) SubGetMessage(vt13_sub, &vt13_data);

    System_State_Report_Remote(vt13_data.offline.is_online || dbus_data.offline.is_online);
    // 周期性刷新底盘反馈在线标志，便于调试时直接观察 B2B_Chassis_Feedback.is_online。
    (void)DualBoard_Chassis_Feedback_Is_Online();

    if (cmd_sys_state.global_mode == GLOBAL_SAFE_LOCK ||
        cmd_sys_state.global_mode == GLOBAL_MODULE_ERROR ||
        cmd_sys_state.global_mode == GLOBAL_STANDBY) {
        // 发送安全帧后立即返回，避免后面的输入计算覆盖 0 速度。
        Cmd_Handle_Safe_Mode();
        Cmd_Update_Chassis_Debug();
        PubPushMessage(chassis_cmd_pub, &chassis_cmd);
        PubPushMessage(gimbal_cmd_pub, &gimbal_cmd);
        PubPushMessage(shoot_cmd_pub, &shoot_cmd);
        Cmd_DualBoard_Sync();
        return;
    }

    if (dbus_data.Ctrl_Mode == 1) {
        Cmd_Update_Mouse_Key();
    } else {
        Cmd_Update_Remote_Ctrl();
    }

    Cmd_Update_Chassis_Debug();

    PubPushMessage(chassis_cmd_pub, &chassis_cmd);
    PubPushMessage(gimbal_cmd_pub, &gimbal_cmd);
    PubPushMessage(shoot_cmd_pub, &shoot_cmd);

    Cmd_DualBoard_Sync();
}

static void Cmd_Handle_Safe_Mode(void)
{
    chassis_cmd.mode = CHASSIS_CMD_SAFE;
    gimbal_cmd.mode = GIMBAL_CMD_SAFE;
    shoot_cmd.mode = SHOOT_CMD_SAFE;

    chassis_cmd.target_vx = 0.0f;
    chassis_cmd.target_vy = 0.0f;
    chassis_cmd.target_vw = 0.0f;
    chassis_cmd.offset_angle = 0.0f;

    gimbal_cmd.target_pitch = 0.0f;
    gimbal_cmd.target_yaw = 0.0f;

    shoot_cmd.friction_rpm = 0.0f;
    shoot_cmd.trigger_single = false;
    shoot_cmd.trigger_auto = false;
}

static void Cmd_Update_Remote_Ctrl(void)
{
    // 第一版移植保留摇杆直接映射到底盘速度，方便上车调通。
    // target_vx/vy 单位为 m/s，target_vw 打包进串口帧前单位为 rad/s。
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
    // 键鼠模式保留新框架命令风格，后续方便做 PC 控制测试。
    chassis_cmd.target_vx = (float)(dbus_data.KeyBoard.W - dbus_data.KeyBoard.S) * KB_WASD_COEF;
    chassis_cmd.target_vy = (float)(dbus_data.KeyBoard.D - dbus_data.KeyBoard.A) * KB_WASD_COEF;

    float active_vw = (float)(dbus_data.KeyBoard.E - dbus_data.KeyBoard.Q) * 3.0f +
                      dbus_data.Mouse.X_Flt * RC_ROCKER_VW_COEF;

    if (dbus_data.KeyBoard.Shift) {
        chassis_cmd.mode = CHASSIS_CMD_SPIN;
        chassis_cmd.target_vw = 5.0f;
    } else {
        chassis_cmd.mode = CHASSIS_CMD_FREE;
        chassis_cmd.target_vw = active_vw;
    }
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
    // USART10 交叉连接到底盘板 USART10。线上协议只传整数:
    // m/s -> mm/s，rad/s -> mrad/s。
    DualBoard_Send_Chassis(&huart10,
                           Cmd_To_DualBoard_Mode(chassis_cmd.mode),
                           chassis_cmd.target_vx * 1000.0f,
                           chassis_cmd.target_vy * 1000.0f,
                           chassis_cmd.target_vw * 1000.0f);
}

static DualBoard_Chassis_Mode_e Cmd_To_DualBoard_Mode(Chassis_Mode_e mode)
{
    if (mode == CHASSIS_CMD_SAFE) return DUALBOARD_CHASSIS_SAFE;
    if (mode == CHASSIS_CMD_SPIN) return DUALBOARD_CHASSIS_SPIN;
    return DUALBOARD_CHASSIS_FREE;
}
