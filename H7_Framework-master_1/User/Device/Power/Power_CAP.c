#include "Power_CAP.h"
#include <string.h>

Cap_t cap;
int open_cap_flag = 0;

/**
 * @brief 电容接收数据解算
 */
void Power_Cap_Rx(void *instance, uint8_t *rx_buf)
{
    if (instance == NULL || rx_buf == NULL) return;

    CapRxData_t *cap_ptr = instance;

    if (rx_buf[7] == 0xAA)
    {
        cap_ptr->offline.last_feed_tick = HAL_GetTick();
        cap_ptr->cap_key   = rx_buf[0];
        cap_ptr->cap_state = rx_buf[1];

        uint16_t raw_voltage = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];
        uint16_t raw_power   = ((uint16_t)rx_buf[4] << 8) | rx_buf[5];

        cap_ptr->bat_voltage = (float)raw_voltage / 100.0f;
        cap_ptr->nowPower    = (float)raw_power / 100.0f;

        cap_ptr->Cap_Capacity = rx_buf[6];
        cap_ptr->check_code = rx_buf[7];
    }
}

void Power_Cap_Tx(hcan_t *hcan, uint16_t can_id, bool enable_boost, float power_limit, uint8_t cur_buffer, bool is_alive) {
    CapSetData_t tx_data = {0};

    tx_data.Control.power_key     = enable_boost ? 1 : 0;
    tx_data.Control.capPowerLimit = (uint8_t)power_limit;
    tx_data.Control.buffer_now    = cur_buffer;
    tx_data.Control.robot_state   = (is_alive > 0) ? 1 : 0;
    tx_data.Control.check_code    = 0xAA;

    FDCAN_Send_Msg(hcan, can_id, tx_data.raw_data, 8);
}