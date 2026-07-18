#ifndef H7_FRAMEWORK_CUSTOM_CONTROLLER_LINK_H
#define H7_FRAMEWORK_CUSTOM_CONTROLLER_LINK_H

#include <stdint.h>

#define CUSTOM_CONTROLLER_FRAME_LENGTH 39U

typedef struct {
    int16_t controller_joint_mrad[6];
    uint8_t controller_clamp;
    uint8_t controller_main_switch;
    uint8_t controller_sequence;
    uint32_t rx_count;
    uint32_t rx_crc_error_count;
    uint32_t rx_format_error_count;
    uint32_t last_rx_ms;

    float arm_position_rad[6];
    uint8_t arm_online_mask;
    uint8_t arm_ready;
    uint8_t tx_sequence;
    uint8_t tx_frame[CUSTOM_CONTROLLER_FRAME_LENGTH];
    uint32_t tx_count;
    uint32_t tx_busy_count;
    uint32_t tx_error_count;
    uint32_t last_tx_ms;
} Custom_Controller_Link_Debug_t;

extern volatile Custom_Controller_Link_Debug_t custom_controller_link_debug;

void Custom_Controller_Link_Init(void);
void Custom_Controller_Link_Update(void);
void Custom_Controller_Link_Rx_Callback(uint8_t *data, void *device_ptr, uint16_t size);

#endif
