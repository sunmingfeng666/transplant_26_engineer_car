#include "Power_Meter.h"
#include "stm32h7xx_hal.h"

Power_Meter_t power_meter;

/**
 * @brief 解析功率计 0x605 标准帧。
 * @note 数据格式沿用老车：分流电压、母线电压和电流均为大端有符号整数，缩放系数为 1000。
 */
void Power_Meter_Rx(void *instance, uint8_t *rx_data)
{
    if (instance == NULL || rx_data == NULL) return;

    Power_Meter_t *power = (Power_Meter_t *)instance;
    const int16_t raw_shunt = (int16_t)(((uint16_t)rx_data[0] << 8) | rx_data[1]);
    const int16_t raw_bus   = (int16_t)(((uint16_t)rx_data[2] << 8) | rx_data[3]);
    const int16_t raw_curr  = (int16_t)(((uint16_t)rx_data[4] << 8) | rx_data[5]);

    power->shunt_volt = (float)raw_shunt / 1000.0f;
    power->bus_volt   = (float)raw_bus / 1000.0f;
    power->current    = (float)raw_curr / 1000.0f;
    power->power      = power->bus_volt * power->current;
    power->offline.last_feed_tick = HAL_GetTick();
}
