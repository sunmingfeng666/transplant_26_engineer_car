#include "Robot_Cmd.h"

#include "Message_Center.h"
#include "System_State.h"
#include "DBUS.h"
#include "VT13.h"
#include "Comm_DualBoard.h"
#include "Picture_Servo.h"
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
static Publisher_t *gimbal_cmd_pub;
static Publisher_t *shoot_cmd_pub;
static Publisher_t *picture_cmd_pub;

static System_State_t cmd_sys_state;
static DBUS_Typedef dbus_data;
static VT13_Typedef vt13_data;

static Chassis_Cmd_t chassis_cmd = {0};
static Gimbal_Cmd_t gimbal_cmd = {0};
static Shoot_Cmd_t shoot_cmd = {0};
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
static void Cmd_Update_Remote_Ctrl(void);
static void Cmd_Update_Mouse_Key(void);
static void Cmd_Update_Picture_Ctrl(void);
static void Cmd_Update_Chassis_Debug(void);
static void Cmd_DualBoard_Sync(void);
static DualBoard_Chassis_Mode_e Cmd_To_DualBoard_Mode(Chassis_Mode_e mode);
static int32_t Cmd_Limit_Int32(int32_t value, int32_t min_value, int32_t max_value);
static uint16_t Cmd_Limit_Servo_Us(int32_t value);
static uint8_t KeyActive(uint8_t key_state);

void Robot_Cmd_Init(void)
{
    sys_state_sub = SubRegister("system_state", sizeof(System_State_t));
    dbus_sub = SubRegister("dbus_data", sizeof(DBUS_Typedef));
    vt13_sub = SubRegister("vt13_data", sizeof(VT13_Typedef));

    chassis_cmd_pub = PubRegister("chassis_cmd", &chassis_cmd, sizeof(Chassis_Cmd_t));
    gimbal_cmd_pub = PubRegister("gimbal_cmd", &gimbal_cmd, sizeof(Gimbal_Cmd_t));
    shoot_cmd_pub = PubRegister("shoot_cmd", &shoot_cmd, sizeof(Shoot_Cmd_t));
    picture_cmd_pub = PubRegister("picture_cmd", &picture_cmd, sizeof(Picture_Cmd_t));

    Picture_Servo_Init();
}

void Robot_Cmd_Update(void)
{
    if (sys_state_sub) SubGetMessage(sys_state_sub, &cmd_sys_state);
    if (dbus_sub) SubGetMessage(dbus_sub, &dbus_data);
    if (vt13_sub) SubGetMessage(vt13_sub, &vt13_data);

    System_State_Report_Remote(vt13_data.offline.is_online || dbus_data.offline.is_online);
    (void)DualBoard_Chassis_Feedback_Is_Online();

    if (cmd_sys_state.global_mode == GLOBAL_SAFE_LOCK ||
        cmd_sys_state.global_mode == GLOBAL_MODULE_ERROR ||
        cmd_sys_state.global_mode == GLOBAL_STANDBY) {
        Cmd_Handle_Safe_Mode();
        Cmd_Update_Chassis_Debug();
        PubPushMessage(chassis_cmd_pub, &chassis_cmd);
        PubPushMessage(gimbal_cmd_pub, &gimbal_cmd);
        PubPushMessage(shoot_cmd_pub, &shoot_cmd);
        PubPushMessage(picture_cmd_pub, &picture_cmd);
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
    PubPushMessage(picture_cmd_pub, &picture_cmd);

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

    picture_cmd.enable = 0U;
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
    DualBoard_Send_Engineer(&huart10,
                            Cmd_To_DualBoard_Mode(chassis_cmd.mode),
                            chassis_cmd.target_vx * 1000.0f,
                            chassis_cmd.target_vy * 1000.0f,
                            chassis_cmd.target_vw * 1000.0f,
                            picture_cmd.lift,
                            picture_cmd.transverse);
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
