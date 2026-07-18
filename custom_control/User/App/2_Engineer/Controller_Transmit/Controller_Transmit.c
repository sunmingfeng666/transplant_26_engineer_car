#include "Controller_Transmit.h"

#include <stdbool.h>
#include <string.h>

#include "CRC_DJI.h"
#include "DM_Motor.h"
#include "DJI_Motor.h"
#include "Robot_Config.h"
#include "fdcan.h"
#include "usart.h"

#define CONTROLLER_FRAME_SOF             0xA5U
#define CONTROLLER_FRAME_CMD_CONTROL     0x0302U
#define CONTROLLER_FRAME_CMD_FEEDBACK    0x0309U
#define CONTROLLER_PAYLOAD_LENGTH        30U
#define CONTROLLER_ALL_JOINTS_ONLINE     0x3FU
#define CONTROLLER_SEND_PERIOD_MS        40U
#define CONTROLLER_FEEDBACK_TIMEOUT_MS   250U
#define CONTROLLER_HOLD_RAMP_MS          2000U
#define CONTROLLER_J1_PROBE_PERIOD_MS     10U
#define CONTROLLER_ENCODER_FULL_SCALE    8191.0f
#define CONTROLLER_DJI2006_REDUCTION     36.0f
#define CONTROLLER_TWO_PI                6.2831853071795864769f
#define CONTROLLER_J1_DM_ID              0x01U

typedef struct __attribute__((packed)) {
    int16_t joint_mrad[CONTROLLER_JOINT_COUNT];
    uint8_t clamp;
    uint8_t main_switch;
    int8_t reserved[16];
} Controller_Command_Payload_t;

typedef union {
    struct __attribute__((packed)) {
        uint8_t sof;
        uint16_t data_length;
        uint8_t sequence;
        uint8_t crc8;
        uint16_t command_id;
        Controller_Command_Payload_t payload;
        uint16_t crc16;
    } fields;
    uint8_t bytes[CONTROLLER_TX_FRAME_LENGTH];
} Controller_Command_Frame_t;

typedef struct {
    float joint_rad[CONTROLLER_JOINT_COUNT];
    uint8_t arm_online_mask;
    uint8_t arm_ready;
    uint8_t sequence;
    uint32_t last_rx_ms;
} Controller_Feedback_State_t;

_Static_assert(sizeof(Controller_Command_Payload_t) == CONTROLLER_PAYLOAD_LENGTH,
               "controller payload must be 30 bytes");
_Static_assert(sizeof(Controller_Command_Frame_t) == CONTROLLER_TX_FRAME_LENGTH,
               "controller frame must be 39 bytes");

/* 车臂 HOME 姿态；控制器六轴首次上线的位置被定义为同一个姿态。 */
static const float controller_home_pose[CONTROLLER_JOINT_COUNT] = {
    -0.8310f, -0.0753f, 0.0811f, 1.8480f, -0.8379f, -2.6858f
};

volatile uint8_t controller_main_switch_manual = 0U;
volatile uint8_t controller_hold_enable_manual = 0U;
volatile uint8_t controller_standalone_hold_enable_manual = 1U;
volatile uint8_t controller_j1_test_enable_manual = 0U;
volatile float controller_hold_current_limit = 500.0f;
volatile float controller_hold_j1_speed = 0.20f;
volatile float controller_hold_kp[CONTROLLER_JOINT_COUNT] = {
    0.0f, 0.35f, 0.35f, 0.35f, 0.28f, 0.28f
};
volatile float controller_hold_kd[CONTROLLER_JOINT_COUNT] = {
    0.0f, 2.0f, 2.0f, 2.0f, 1.5f, 1.5f
};
Controller_Transmit_Debug_t controller_tx_debug;

/* DMA1 无法访问 DTCM，收发缓冲区由 Robot_Config/本文件放在 D2 SRAM。 */
static Controller_Command_Frame_t controller_tx_frame __attribute__((section(".RAM_D2")));
static Controller_Feedback_State_t feedback_state;
static uint32_t last_send_tick;
static float encoder_boot[CONTROLLER_JOINT_COUNT];
static uint8_t zero_captured_mask;
static bool hold_was_active;
static bool standalone_hold_was_active;
static bool j1_test_was_active;
static uint32_t j1_disabled_probe_last_ms;
static float j1_test_target_motor;
static uint32_t hold_ramp_start_ms;
static float hold_ramp_start_rad[CONTROLLER_JOINT_COUNT];
static float last_dji_error[CONTROLLER_JOINT_COUNT];

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static float limit_float(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static bool joint_is_online(uint8_t joint)
{
    switch (joint) {
        case 0U: return engineer_custom_motors.J1_DM4310.offline.is_online;
        case 1U: return engineer_custom_motors.J2_DJI2006.offline.is_online;
        case 2U: return engineer_custom_motors.J3_DJI3508.offline.is_online;
        case 3U: return engineer_custom_motors.J4_DJI3508.offline.is_online;
        case 4U: return engineer_custom_motors.J5_DJI2006.offline.is_online;
        case 5U: return engineer_custom_motors.J6_DJI2006.offline.is_online;
        default: return false;
    }
}

static int32_t joint_encoder(uint8_t joint)
{
    switch (joint) {
        case 1U: return engineer_custom_motors.J2_DJI2006.Angle_Infinite;
        case 2U: return engineer_custom_motors.J3_DJI3508.Angle_Infinite;
        case 3U: return engineer_custom_motors.J4_DJI3508.Angle_Infinite;
        case 4U: return engineer_custom_motors.J5_DJI2006.Angle_Infinite;
        case 5U: return engineer_custom_motors.J6_DJI2006.Angle_Infinite;
        default: return 0;
    }
}

static uint8_t update_online_mask(void)
{
    uint8_t mask = 0U;
    for (uint8_t joint = 0U; joint < CONTROLLER_JOINT_COUNT; ++joint) {
        if (joint_is_online(joint)) mask |= (uint8_t)(1U << joint);
    }
    return mask;
}

static void capture_boot_positions(void)
{
    for (uint8_t joint = 0U; joint < CONTROLLER_JOINT_COUNT; ++joint) {
        const uint8_t bit = (uint8_t)(1U << joint);
        if ((zero_captured_mask & bit) != 0U || !joint_is_online(joint)) continue;
        encoder_boot[joint] = (joint == 0U)
                                  ? engineer_custom_motors.J1_DM4310.pos
                                  : (float)joint_encoder(joint);
        zero_captured_mask |= bit;
    }
}

static void update_joint_mapping(void)
{
    const float encoder_to_rad = CONTROLLER_TWO_PI / CONTROLLER_ENCODER_FULL_SCALE;
    controller_tx_debug.online_mask = update_online_mask();
    capture_boot_positions();
    controller_tx_debug.zero_captured_mask = zero_captured_mask;

    controller_tx_debug.encoder_raw[0] = engineer_custom_motors.J1_DM4310.p_int;
    for (uint8_t joint = 1U; joint < CONTROLLER_JOINT_COUNT; ++joint) {
        controller_tx_debug.encoder_raw[joint] = joint_encoder(joint);
    }
    memcpy(controller_tx_debug.encoder_boot, encoder_boot, sizeof(encoder_boot));

    controller_tx_debug.joint_rad[0] = controller_home_pose[0] +
        (engineer_custom_motors.J1_DM4310.pos - encoder_boot[0]);
    controller_tx_debug.joint_rad[1] = controller_home_pose[1] +
        ((float)joint_encoder(1U) - encoder_boot[1]) * encoder_to_rad / CONTROLLER_DJI2006_REDUCTION;
    controller_tx_debug.joint_rad[2] = controller_home_pose[2] -
        ((float)joint_encoder(2U) - encoder_boot[2]) * encoder_to_rad;
    controller_tx_debug.joint_rad[3] = controller_home_pose[3] +
        ((float)joint_encoder(3U) - encoder_boot[3]) * encoder_to_rad;
    controller_tx_debug.joint_rad[4] = controller_home_pose[4] +
        ((float)joint_encoder(4U) - encoder_boot[4]) * encoder_to_rad / CONTROLLER_DJI2006_REDUCTION;
    controller_tx_debug.joint_rad[5] = controller_home_pose[5] -
        ((float)joint_encoder(5U) - encoder_boot[5]) * encoder_to_rad;

    for (uint8_t joint = 0U; joint < CONTROLLER_JOINT_COUNT; ++joint) {
        const float value_mrad = limit_float(controller_tx_debug.joint_rad[joint] * 1000.0f,
                                              -32768.0f, 32767.0f);
        controller_tx_debug.joint_mrad[joint] = (int16_t)value_mrad;
    }

    /* PA10已作为USART1_RX，夹爪输入本阶段固定发送0。 */
    controller_tx_debug.clamp_raw = 0U;
    controller_tx_debug.clamp_debounced = 0U;
    controller_tx_debug.main_switch_manual = controller_main_switch_manual ? 1U : 0U;
    controller_tx_debug.main_switch_effective =
        (controller_tx_debug.main_switch_manual != 0U &&
         controller_tx_debug.online_mask == CONTROLLER_ALL_JOINTS_ONLINE &&
         zero_captured_mask == CONTROLLER_ALL_JOINTS_ONLINE) ? 1U : 0U;
}

void Controller_Feedback_Rx_Callback(uint8_t *data, void *device_ptr, uint16_t size)
{
    (void)device_ptr;
    if (data == NULL || size != CONTROLLER_RX_FRAME_LENGTH) {
        ++controller_tx_debug.rx_format_error_count;
        return;
    }
    if (data[0] != CONTROLLER_FRAME_SOF ||
        read_u16_le(&data[1]) != CONTROLLER_PAYLOAD_LENGTH ||
        read_u16_le(&data[5]) != CONTROLLER_FRAME_CMD_FEEDBACK) {
        ++controller_tx_debug.rx_format_error_count;
        return;
    }
    if (!Verify_CRC8_Check_Sum(data, 5U) ||
        !Verify_CRC16_Check_Sum(data, CONTROLLER_RX_FRAME_LENGTH)) {
        ++controller_tx_debug.rx_crc_error_count;
        return;
    }

    Controller_Feedback_State_t next = {0};
    next.arm_ready = data[7] ? 1U : 0U;
    for (uint8_t joint = 0U; joint < CONTROLLER_JOINT_COUNT; ++joint) {
        memcpy(&next.joint_rad[joint], &data[8U + 4U * joint], sizeof(float));
    }
    next.arm_online_mask = data[32];
    next.sequence = data[3];
    next.last_rx_ms = HAL_GetTick();
    feedback_state = next;

    memcpy(controller_tx_debug.rx_frame, data, CONTROLLER_RX_FRAME_LENGTH);
    controller_tx_debug.rx_sequence = next.sequence;
    controller_tx_debug.last_rx_ms = next.last_rx_ms;
    ++controller_tx_debug.rx_count;
}

static Controller_Feedback_State_t feedback_snapshot(void)
{
    Controller_Feedback_State_t snapshot;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    snapshot = feedback_state;
    if (primask == 0U) __enable_irq();
    return snapshot;
}

static float joint_target_to_motor(uint8_t joint, float joint_rad)
{
    const float delta = joint_rad - controller_home_pose[joint];
    const float rad_to_encoder = CONTROLLER_ENCODER_FULL_SCALE / CONTROLLER_TWO_PI;
    switch (joint) {
        case 0U: return encoder_boot[0] + delta;
        case 1U: return encoder_boot[1] + delta * rad_to_encoder * CONTROLLER_DJI2006_REDUCTION;
        case 2U: return encoder_boot[2] - delta * rad_to_encoder;
        case 3U: return encoder_boot[3] + delta * rad_to_encoder;
        case 4U: return encoder_boot[4] + delta * rad_to_encoder * CONTROLLER_DJI2006_REDUCTION;
        case 5U: return encoder_boot[5] - delta * rad_to_encoder;
        default: return 0.0f;
    }
}

static int16_t calc_dji_output(uint8_t joint, float motor_target)
{
    const float error = motor_target - (float)joint_encoder(joint);
    const float output = controller_hold_kp[joint] * error +
                         controller_hold_kd[joint] * (error - last_dji_error[joint]);
    last_dji_error[joint] = error;
    const float current_limit = limit_float(controller_hold_current_limit, 0.0f, 10000.0f);
    const float limited = limit_float(output, -current_limit, current_limit);
    return (int16_t)limited;
}

static void stop_hold_outputs(void)
{
    if (!hold_was_active) return;
    Motor_Mode(&hfdcan2, CONTROLLER_J1_DM_ID, POS_MODE, DM_CMD_RESET_MODE);
    DJI_Motor_Send(&hfdcan1, 0x200U, 0, 0, 0, 0);
    DJI_Motor_Send(&hfdcan2, 0x1FFU, 0, 0, 0, 0);
    memset(last_dji_error, 0, sizeof(last_dji_error));
    memset(controller_tx_debug.output_current, 0, sizeof(controller_tx_debug.output_current));
    hold_was_active = false;
}

static void stop_j1_test_output(void)
{
    if (!j1_test_was_active) return;
    Motor_Mode(&hfdcan2, CONTROLLER_J1_DM_ID, POS_MODE, DM_CMD_RESET_MODE);
    j1_test_was_active = false;
    controller_tx_debug.j1_test_effective = 0U;
}

static void stop_standalone_hold_outputs(void)
{
    if (!standalone_hold_was_active) return;
    Motor_Mode(&hfdcan2, CONTROLLER_J1_DM_ID, POS_MODE, DM_CMD_RESET_MODE);
    DJI_Motor_Send(&hfdcan1, 0x200U, 0, 0, 0, 0);
    DJI_Motor_Send(&hfdcan2, 0x1FFU, 0, 0, 0, 0);
    memset(last_dji_error, 0, sizeof(last_dji_error));
    memset(controller_tx_debug.output_current, 0,
           sizeof(controller_tx_debug.output_current));
    standalone_hold_was_active = false;
    controller_tx_debug.standalone_hold_effective = 0U;
}

static void update_j1_disabled_feedback_probe(uint32_t now)
{
    if (controller_tx_debug.j1_test_effective != 0U ||
        controller_tx_debug.standalone_hold_effective != 0U ||
        controller_tx_debug.hold_effective != 0U) {
        return;
    }
    if ((uint32_t)(now - j1_disabled_probe_last_ms) < CONTROLLER_J1_PROBE_PERIOD_MS) {
        return;
    }

    j1_disabled_probe_last_ms = now;
    /* 0xFD 只保持失能，用它触发达妙回复位置，不产生保持力。 */
    Motor_Mode(&hfdcan2, CONTROLLER_J1_DM_ID, POS_MODE, DM_CMD_RESET_MODE);
    ++controller_tx_debug.j1_disabled_probe_count;
}

static void update_position_hold(uint32_t now)
{
    const Controller_Feedback_State_t feedback = feedback_snapshot();
    const uint8_t feedback_online =
        (feedback.last_rx_ms != 0U &&
         (uint32_t)(now - feedback.last_rx_ms) <= CONTROLLER_FEEDBACK_TIMEOUT_MS) ? 1U : 0U;

    memcpy(controller_tx_debug.arm_feedback_rad, feedback.joint_rad,
           sizeof(controller_tx_debug.arm_feedback_rad));
    controller_tx_debug.arm_online_mask = feedback.arm_online_mask;
    controller_tx_debug.arm_ready = feedback.arm_ready;
    controller_tx_debug.feedback_online = feedback_online;
    controller_tx_debug.hold_enable_manual = controller_hold_enable_manual ? 1U : 0U;
    controller_tx_debug.standalone_hold_enable_manual =
        controller_standalone_hold_enable_manual ? 1U : 0U;
    controller_tx_debug.j1_test_enable_manual =
        controller_j1_test_enable_manual ? 1U : 0U;

    /*
     * 六轴独立保持测试：不需要车板回传。
     * encoder_boot 对应 controller_home_pose，因此目标是上电时的六轴实际姿态。
     */
    if (controller_tx_debug.standalone_hold_enable_manual != 0U) {
        const uint8_t standalone_effective =
            (controller_tx_debug.online_mask == CONTROLLER_ALL_JOINTS_ONLINE &&
             zero_captured_mask == CONTROLLER_ALL_JOINTS_ONLINE) ? 1U : 0U;

        controller_tx_debug.hold_effective = 0U;
        controller_tx_debug.j1_test_effective = 0U;
        controller_tx_debug.standalone_hold_effective = standalone_effective;
        stop_j1_test_output();
        stop_hold_outputs();

        if (standalone_effective == 0U) {
            stop_standalone_hold_outputs();
            controller_tx_debug.hold_ramp = 0.0f;
            return;
        }

        if (!standalone_hold_was_active) {
            memcpy(hold_ramp_start_rad, controller_tx_debug.joint_rad,
                   sizeof(hold_ramp_start_rad));
            memset(last_dji_error, 0, sizeof(last_dji_error));
            hold_ramp_start_ms = now;
            Motor_Mode(&hfdcan2, CONTROLLER_J1_DM_ID, POS_MODE, DM_CMD_CLEAR_ERROR);
            Motor_Mode(&hfdcan2, CONTROLLER_J1_DM_ID, POS_MODE, DM_CMD_MOTOR_MODE);
            standalone_hold_was_active = true;
        }

        const float ramp = limit_float((float)(now - hold_ramp_start_ms) /
                                       (float)CONTROLLER_HOLD_RAMP_MS, 0.0f, 1.0f);
        controller_tx_debug.hold_ramp = ramp;
        for (uint8_t joint = 0U; joint < CONTROLLER_JOINT_COUNT; ++joint) {
            const float target_rad = hold_ramp_start_rad[joint] +
                (controller_home_pose[joint] - hold_ramp_start_rad[joint]) * ramp;
            controller_tx_debug.hold_target_rad[joint] = target_rad;
            controller_tx_debug.hold_error_rad[joint] =
                target_rad - controller_tx_debug.joint_rad[joint];
            controller_tx_debug.motor_target[joint] =
                joint_target_to_motor(joint, target_rad);
        }

        Pos_Speed_Ctrl(&hfdcan2, CONTROLLER_J1_DM_ID,
                       controller_tx_debug.motor_target[0],
                       limit_float(controller_hold_j1_speed, 0.0f, 0.60f));
        for (uint8_t joint = 1U; joint < CONTROLLER_JOINT_COUNT; ++joint) {
            controller_tx_debug.output_current[joint] =
                calc_dji_output(joint, controller_tx_debug.motor_target[joint]);
        }
        DJI_Motor_Send(&hfdcan1, 0x200U, 0,
                       controller_tx_debug.output_current[1],
                       controller_tx_debug.output_current[2],
                       controller_tx_debug.output_current[3]);
        DJI_Motor_Send(&hfdcan2, 0x1FFU,
                       controller_tx_debug.output_current[4],
                       controller_tx_debug.output_current[5], 0, 0);
        return;
    }

    stop_standalone_hold_outputs();
    controller_tx_debug.standalone_hold_effective = 0U;

    /*
     * J1 单轴独立测试优先于车臂角度跟随：
     * 只要求本地 J1 在线，并在手动打开的瞬间把当前位置锁定为目标。
     */
    if (controller_tx_debug.j1_test_enable_manual != 0U) {
        const uint8_t j1_test_effective =
            (joint_is_online(0U) && (zero_captured_mask & 0x01U) != 0U) ? 1U : 0U;

        controller_tx_debug.hold_effective = 0U;
        controller_tx_debug.hold_ramp = 0.0f;
        controller_tx_debug.j1_test_effective = j1_test_effective;
        stop_hold_outputs();

        if (j1_test_effective == 0U) {
            if (j1_test_was_active) {
                stop_j1_test_output();
            }
            return;
        }

        if (!j1_test_was_active) {
            j1_test_target_motor = engineer_custom_motors.J1_DM4310.pos;
            Motor_Mode(&hfdcan2, CONTROLLER_J1_DM_ID, POS_MODE, DM_CMD_CLEAR_ERROR);
            Motor_Mode(&hfdcan2, CONTROLLER_J1_DM_ID, POS_MODE, DM_CMD_MOTOR_MODE);
            j1_test_was_active = true;
        }

        controller_tx_debug.j1_test_target_motor = j1_test_target_motor;
        controller_tx_debug.motor_target[0] = j1_test_target_motor;
        controller_tx_debug.hold_target_rad[0] = controller_home_pose[0] +
            (j1_test_target_motor - encoder_boot[0]);
        controller_tx_debug.hold_error_rad[0] =
            controller_tx_debug.hold_target_rad[0] - controller_tx_debug.joint_rad[0];
        memset(controller_tx_debug.output_current, 0,
               sizeof(controller_tx_debug.output_current));

        Pos_Speed_Ctrl(&hfdcan2, CONTROLLER_J1_DM_ID, j1_test_target_motor,
                       limit_float(controller_hold_j1_speed, 0.0f, 0.60f));
        return;
    }

    stop_j1_test_output();
    controller_tx_debug.j1_test_effective = 0U;

    const uint8_t hold_effective =
        (controller_tx_debug.hold_enable_manual != 0U &&
         feedback_online != 0U && feedback.arm_ready != 0U &&
         feedback.arm_online_mask == CONTROLLER_ALL_JOINTS_ONLINE &&
         controller_tx_debug.online_mask == CONTROLLER_ALL_JOINTS_ONLINE &&
         zero_captured_mask == CONTROLLER_ALL_JOINTS_ONLINE) ? 1U : 0U;
    controller_tx_debug.hold_effective = hold_effective;

    if (!hold_effective) {
        controller_tx_debug.hold_ramp = 0.0f;
        stop_hold_outputs();
        return;
    }

    if (!hold_was_active) {
        memcpy(hold_ramp_start_rad, controller_tx_debug.joint_rad,
               sizeof(hold_ramp_start_rad));
        memset(last_dji_error, 0, sizeof(last_dji_error));
        hold_ramp_start_ms = now;
        Motor_Mode(&hfdcan2, CONTROLLER_J1_DM_ID, POS_MODE, DM_CMD_CLEAR_ERROR);
        Motor_Mode(&hfdcan2, CONTROLLER_J1_DM_ID, POS_MODE, DM_CMD_MOTOR_MODE);
        hold_was_active = true;
    }

    const float ramp = limit_float((float)(now - hold_ramp_start_ms) /
                                   (float)CONTROLLER_HOLD_RAMP_MS, 0.0f, 1.0f);
    controller_tx_debug.hold_ramp = ramp;
    for (uint8_t joint = 0U; joint < CONTROLLER_JOINT_COUNT; ++joint) {
        const float target_rad = hold_ramp_start_rad[joint] +
            (feedback.joint_rad[joint] - hold_ramp_start_rad[joint]) * ramp;
        controller_tx_debug.hold_target_rad[joint] = target_rad;
        controller_tx_debug.hold_error_rad[joint] = target_rad - controller_tx_debug.joint_rad[joint];
        controller_tx_debug.motor_target[joint] = joint_target_to_motor(joint, target_rad);
    }

    Pos_Speed_Ctrl(&hfdcan2, CONTROLLER_J1_DM_ID,
                   controller_tx_debug.motor_target[0],
                   limit_float(controller_hold_j1_speed, 0.0f, 0.60f));
    for (uint8_t joint = 1U; joint < CONTROLLER_JOINT_COUNT; ++joint) {
        controller_tx_debug.output_current[joint] =
            calc_dji_output(joint, controller_tx_debug.motor_target[joint]);
    }
    DJI_Motor_Send(&hfdcan1, 0x200U, 0,
                   controller_tx_debug.output_current[1],
                   controller_tx_debug.output_current[2],
                   controller_tx_debug.output_current[3]);
    DJI_Motor_Send(&hfdcan2, 0x1FFU,
                   controller_tx_debug.output_current[4],
                   controller_tx_debug.output_current[5], 0, 0);
}

static void build_command_frame(void)
{
    controller_tx_frame.fields.sof = CONTROLLER_FRAME_SOF;
    controller_tx_frame.fields.data_length = CONTROLLER_PAYLOAD_LENGTH;
    controller_tx_frame.fields.sequence = (uint8_t)(controller_tx_frame.fields.sequence + 1U);
    controller_tx_frame.fields.crc8 = 0U;
    Append_CRC8_Check_Sum(controller_tx_frame.bytes, 5U);
    controller_tx_frame.fields.command_id = CONTROLLER_FRAME_CMD_CONTROL;
    memcpy(controller_tx_frame.fields.payload.joint_mrad,
           controller_tx_debug.joint_mrad,
           sizeof(controller_tx_frame.fields.payload.joint_mrad));
    controller_tx_frame.fields.payload.clamp = 0U;
    controller_tx_frame.fields.payload.main_switch = controller_tx_debug.main_switch_effective;
    memset(controller_tx_frame.fields.payload.reserved, 0,
           sizeof(controller_tx_frame.fields.payload.reserved));
    controller_tx_frame.fields.crc16 = 0U;
    Append_CRC16_Check_Sum(controller_tx_frame.bytes, CONTROLLER_TX_FRAME_LENGTH);
    controller_tx_debug.sequence = controller_tx_frame.fields.sequence;
    memcpy(controller_tx_debug.frame, controller_tx_frame.bytes, CONTROLLER_TX_FRAME_LENGTH);
}

void Controller_Transmit_Init(void)
{
    memset(&controller_tx_debug, 0, sizeof(controller_tx_debug));
    memset(&controller_tx_frame, 0, sizeof(controller_tx_frame));
    memset(&feedback_state, 0, sizeof(feedback_state));
    memset(encoder_boot, 0, sizeof(encoder_boot));
    memset(last_dji_error, 0, sizeof(last_dji_error));
    controller_main_switch_manual = 0U;
    controller_hold_enable_manual = 0U;
    /* 当前阶段不接车板：上电后六轴反馈齐全即自动保持 HOME 姿态。 */
    controller_standalone_hold_enable_manual = 1U;
    controller_j1_test_enable_manual = 0U;
    zero_captured_mask = 0U;
    hold_was_active = false;
    standalone_hold_was_active = false;
    j1_test_was_active = false;
    j1_disabled_probe_last_ms = HAL_GetTick();
    j1_test_target_motor = 0.0f;
    last_send_tick = HAL_GetTick();
}

void Controller_Transmit_Update(void)
{
    const uint32_t now = HAL_GetTick();
    update_joint_mapping();
    update_position_hold(now);
    update_j1_disabled_feedback_probe(now);

    if ((uint32_t)(now - last_send_tick) < CONTROLLER_SEND_PERIOD_MS) return;
    last_send_tick = now;
    if (huart1.gState != HAL_UART_STATE_READY) {
        ++controller_tx_debug.busy_count;
        controller_tx_debug.last_hal_status = (uint32_t)HAL_BUSY;
        return;
    }

    build_command_frame();
    const HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(
        &huart1, controller_tx_frame.bytes, CONTROLLER_TX_FRAME_LENGTH);
    controller_tx_debug.last_hal_status = (uint32_t)status;
    if (status == HAL_OK) ++controller_tx_debug.tx_count;
    else if (status == HAL_BUSY) ++controller_tx_debug.busy_count;
    else ++controller_tx_debug.error_count;
}
