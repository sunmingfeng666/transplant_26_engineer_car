#ifndef H7_FRAMEWORK_CONTROLLER_TRANSMIT_H
#define H7_FRAMEWORK_CONTROLLER_TRANSMIT_H

#include <stdint.h>

#define CONTROLLER_JOINT_COUNT       6U
#define CONTROLLER_TX_FRAME_LENGTH  39U
#define CONTROLLER_RX_FRAME_LENGTH  39U

typedef struct {
    float joint_rad[CONTROLLER_JOINT_COUNT];
    int16_t joint_mrad[CONTROLLER_JOINT_COUNT];
    int32_t encoder_raw[CONTROLLER_JOINT_COUNT];
    float encoder_boot[CONTROLLER_JOINT_COUNT];
    uint8_t zero_captured_mask;
    uint8_t online_mask;
    uint8_t clamp_raw;
    uint8_t clamp_debounced;
    uint8_t main_switch_manual;
    uint8_t main_switch_effective;
    uint8_t sequence;
    uint8_t frame[CONTROLLER_TX_FRAME_LENGTH];
    uint32_t tx_count;
    uint32_t busy_count;
    uint32_t error_count;
    uint32_t last_hal_status;

    float arm_feedback_rad[CONTROLLER_JOINT_COUNT];
    uint8_t arm_online_mask;
    uint8_t arm_ready;
    uint8_t feedback_online;
    uint8_t rx_sequence;
    uint8_t rx_frame[CONTROLLER_RX_FRAME_LENGTH];
    uint32_t rx_count;
    uint32_t rx_crc_error_count;
    uint32_t rx_format_error_count;
    uint32_t last_rx_ms;

    uint8_t hold_enable_manual;
    uint8_t hold_effective;
    uint8_t standalone_hold_enable_manual;
    uint8_t standalone_hold_effective;
    uint8_t j1_test_enable_manual;
    uint8_t j1_test_effective;
    uint32_t j1_disabled_probe_count;
    float j1_test_target_motor;
    float hold_ramp;
    float hold_target_rad[CONTROLLER_JOINT_COUNT];
    float hold_error_rad[CONTROLLER_JOINT_COUNT];
    float motor_target[CONTROLLER_JOINT_COUNT];
    int16_t output_current[CONTROLLER_JOINT_COUNT];
} Controller_Transmit_Debug_t;

/* 车臂控制总开关：Ozone手动改为1；本阶段板1只解析，不会把它转发给车臂。 */
extern volatile uint8_t controller_main_switch_manual;
/* 本地位置保持独立使能：Ozone手动改为1后，安全条件全部满足才产生本机保持力。 */
extern volatile uint8_t controller_hold_enable_manual;
/* 六轴独立保持测试：不依赖车板，把上电姿态作为机械臂 HOME 姿态。 */
extern volatile uint8_t controller_standalone_hold_enable_manual;
/* J1 达妙单轴独立测试：不依赖车板回传和其余五轴，打开瞬间锁定当前位置。 */
extern volatile uint8_t controller_j1_test_enable_manual;
extern volatile float controller_hold_current_limit;
extern volatile float controller_hold_j1_speed;
extern volatile float controller_hold_kp[CONTROLLER_JOINT_COUNT];
extern volatile float controller_hold_kd[CONTROLLER_JOINT_COUNT];
extern Controller_Transmit_Debug_t controller_tx_debug;

void Controller_Transmit_Init(void);
void Controller_Transmit_Update(void);
void Controller_Feedback_Rx_Callback(uint8_t *data, void *device_ptr, uint16_t size);

#endif
