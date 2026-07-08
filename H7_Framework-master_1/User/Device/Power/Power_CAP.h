#ifndef H7_FRAMEWORK_POWER_CAP_H
#define H7_FRAMEWORK_POWER_CAP_H

#include <stdint.h>
#include <stdbool.h>
#include "BSP_FDCAN.h"
#include "Offline_Detector.h"

#pragma pack(1)
typedef struct {
    Offline_Check_t offline;
    uint8_t  cap_key;       // FSBB开关状态
    uint8_t  cap_state;     // 状态位 (0:正常, 1:故障)
    float    bat_voltage;   // 电池端电压
    float    nowPower;      // 当前输入功率
    uint8_t  Cap_Capacity;  // 电容容量百分比
    uint8_t  check_code;    // 校验位
} CapRxData_t;

typedef union {
    struct {
        uint8_t power_key;       // 开关控制 (1:放电冲刺, 0:正常充电/待机)
        uint8_t capPowerLimit;   // 给电容板下发的功率限制值 (W)
        uint8_t buffer_now;      // 裁判系统当前剩余缓冲能量
        uint8_t robot_state;     // 机器人存活状态 (1:存活, 0:死亡)
        uint8_t check_code;      // 校验位 (0xAA)
        uint8_t reserved[3];
    } Control;
    uint8_t raw_data[8];
} CapSetData_t;
#pragma pack()

typedef struct {
    CapSetData_t set;
    CapRxData_t  get;
} Cap_t;

/* 外部变量声明 */
extern Cap_t cap;

void Power_Cap_Rx(void *instance, uint8_t *rx_buf);

void Power_Cap_Tx(hcan_t *hcan, uint16_t can_id, bool enable_boost, float power_limit, uint8_t cur_buffer, bool is_alive);

#endif //H7_FRAMEWORK_POWER_CAP_H