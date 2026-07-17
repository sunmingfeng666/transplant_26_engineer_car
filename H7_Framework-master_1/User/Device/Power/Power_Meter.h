#ifndef H7_FRAMEWORK_POWER_METER_H
#define H7_FRAMEWORK_POWER_METER_H

#include <stdint.h>
#include "Offline_Detector.h"

#define POWER_METER_CAN_ID          0x605U
#define POWER_METER_OFFLINE_TIME    100U

// 底盘独立功率计测量数据，由板1的 FDCAN1 接收。
typedef struct {
    Offline_Check_t offline;
    float shunt_volt;
    float bus_volt;
    float current;
    float power;           // 母线电压乘以电流得到的当前实际功率，单位 W
} Power_Meter_t;

extern Power_Meter_t power_meter;

// 回调签名与 BSP_CAN_Callback_t 一致，可直接用于 CAN_RX_NODE。
void Power_Meter_Rx(void *instance, uint8_t *rx_data);

#endif //H7_FRAMEWORK_POWER_METER_H
