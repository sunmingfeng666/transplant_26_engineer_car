#include "Referee.h"
#include <stdbool.h>

Referee_Data_t Referee;
uint8_t Referee_Rx_Buf[2][REFEREE_RXFRAME_LENGTH]__attribute__((section(".RAM_D2")));

static bool Referee_System_Info_Update(uint16_t cmd_id,
                                       const uint8_t *data_ptr,
                                       uint16_t payload_len,
                                       Referee_Data_t *user_data);

void Referee_System_Frame_Update(uint8_t *Buff, void *device_ptr, uint16_t Size)
{
    (void)device_ptr;

    /*
     * UART DMA 每次回调的切分位置不保证落在帧边界上，因此先把数据追加到流缓存，
     * 再统一完成找帧头、长度检查、CRC 校验和断包保留。
     */
    static uint8_t stream_buf[REFEREE_RXFRAME_LENGTH + REFEREE_MAX_PACKET_SIZE];
    static uint16_t stream_len = 0U;

    if (Buff == NULL || Size == 0U || Size > REFEREE_RXFRAME_LENGTH) return;

    if ((uint32_t)stream_len + Size > sizeof(stream_buf)) {
        /* 缓存状态异常时整段丢弃，禁止越界，也避免错误长度长期阻塞后续合法帧。 */
        stream_len = 0U;
    }
    memcpy(&stream_buf[stream_len], Buff, Size);
    stream_len = (uint16_t)(stream_len + Size);

    uint16_t offset = 0U;
    while ((uint16_t)(stream_len - offset) >= FrameHeader_Length) {
        if (stream_buf[offset] != 0xA5U) {
            offset++;
            continue;
        }

        if (!Verify_CRC8_Check_Sum(&stream_buf[offset], FrameHeader_Length)) {
            offset++;
            continue;
        }

        const uint16_t payload_len = (uint16_t)stream_buf[offset + 1U]
                                   | ((uint16_t)stream_buf[offset + 2U] << 8U);
        const uint32_t frame_len_u32 = (uint32_t)payload_len
                                     + FrameHeader_Length + CMDID_Length + CRC16_Length;

        /* 单包上限同时约束声明长度和实际访问范围，异常帧只滑动一个字节重新找头。 */
        if (frame_len_u32 > REFEREE_MAX_PACKET_SIZE ||
            frame_len_u32 < (FrameHeader_Length + CMDID_Length + CRC16_Length)) {
            offset++;
            continue;
        }

        const uint16_t frame_len = (uint16_t)frame_len_u32;
        if ((uint16_t)(stream_len - offset) < frame_len) {
            /* 当前回调数据不足一整帧，保留到下一次 DMA 回调继续拼接。 */
            break;
        }

        if (Verify_CRC16_Check_Sum(&stream_buf[offset], frame_len)) {
            const uint16_t cmd_id = (uint16_t)stream_buf[offset + FrameHeader_Length]
                                  | ((uint16_t)stream_buf[offset + FrameHeader_Length + 1U] << 8U);
            const uint8_t *data_ptr = &stream_buf[offset + FrameHeader_Length + CMDID_Length];

            /* 只有命令已支持且载荷长度正确，才更新数据和在线喂狗时间。 */
            if (Referee_System_Info_Update(cmd_id, data_ptr, payload_len, &Referee)) {
                Referee.offline.last_feed_tick = HAL_GetTick();
            }
            offset = (uint16_t)(offset + frame_len);
            continue;
        }

        offset++;
    }

    if (offset > 0U) {
        const uint16_t remain_len = (uint16_t)(stream_len - offset);
        if (remain_len > 0U) {
            memmove(stream_buf, &stream_buf[offset], remain_len);
        }
        stream_len = remain_len;
    }
}

static bool Referee_System_Info_Update(uint16_t cmd_id,
                                       const uint8_t *data_ptr,
                                       uint16_t payload_len,
                                       Referee_Data_t *user_data)
{
    if (data_ptr == NULL || user_data == NULL) return false;

#define REFEREE_COPY_CASE(command, member, type) \
        case command: \
            if (payload_len != sizeof(type)) return false; \
            memcpy(&user_data->member, data_ptr, sizeof(type)); \
            return true

    switch (cmd_id)
    {
        REFEREE_COPY_CASE(game_state, game_status, game_status_t);
        REFEREE_COPY_CASE(Match_results, game_result, game_result_t);
        REFEREE_COPY_CASE(Robot_HP, game_robot_HP, game_robot_HP_t);
        REFEREE_COPY_CASE(Venue_Events, event_data, event_data_t);
        REFEREE_COPY_CASE(Referee_warning, referee_warning, referee_warning_t);
        REFEREE_COPY_CASE(Dart_fire, dart_info, dart_info_t);
        case Robot_performan:
            if (payload_len != sizeof(robot_status_t)) return false;
            memcpy(&user_data->robot_status, data_ptr, sizeof(robot_status_t));
            user_data->valid_flags |= REFEREE_VALID_ROBOT_STATUS;
            user_data->robot_status_tick = HAL_GetTick();
            return true;
        case time_power:
            if (payload_len != sizeof(power_heat_data_t)) return false;
            memcpy(&user_data->power_heat_data, data_ptr, sizeof(power_heat_data_t));
            user_data->valid_flags |= REFEREE_VALID_POWER_HEAT;
            user_data->power_heat_tick = HAL_GetTick();
            return true;
        REFEREE_COPY_CASE(Robot_location, robot_pos, robot_pos_t);
        REFEREE_COPY_CASE(Robot_buff, buff, buff_t);
        REFEREE_COPY_CASE(Damage_status, hurt_data, hurt_data_t);
        REFEREE_COPY_CASE(time_shooting, shoot_data, shoot_data_t);
        REFEREE_COPY_CASE(Allowable_ammo, projectile_allowance, projectile_allowance_t);
        REFEREE_COPY_CASE(RFID_status, rfid_status, rfid_status_t);
        REFEREE_COPY_CASE(Dart_directives, dart_client_cmd, dart_client_cmd_t);
        REFEREE_COPY_CASE(Ground_location, ground_robot_position, ground_robot_position_t);
        REFEREE_COPY_CASE(Radar_Marking, radar_mark_data, radar_mark_data_t);
        REFEREE_COPY_CASE(Route_Informat, sentry_info, sentry_info_t);
        REFEREE_COPY_CASE(Radar_Informat, radar_info, radar_info_t);
        REFEREE_COPY_CASE(Minimap, map_command, map_command_t);
        default: return false;
    }

#undef REFEREE_COPY_CASE
}


static uint8_t referee_tx_seq = 0;

/**
  * @brief  裁判系统通用发送函数
  * @param  cmd_id: 命令码
  * @param  p_data: 负载数据指针
  * @param  len: 负载数据长度
  */
void Referee_Send_Data(uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
    static uint8_t tx_buf[REFEREE_RXFRAME_LENGTH]; // 发送缓冲区
    uint16_t frame_length = FrameHeader_Length + CMDID_Length + len + CRC16_Length;

    tx_buf[0] = 0xA5;                                  // SOF
    tx_buf[1] = (uint8_t)(len & 0x00FF);               // Data Length LSB
    tx_buf[2] = (uint8_t)((len >> 8) & 0x00FF);        // Data Length MSB
    tx_buf[3] = referee_tx_seq++;                      // Seq
    Append_CRC8_Check_Sum(tx_buf, FrameHeader_Length); // 计算并添加 CRC8

    tx_buf[5] = (uint8_t)(cmd_id & 0x00FF);
    tx_buf[6] = (uint8_t)((cmd_id >> 8) & 0x00FF);

    memcpy(&tx_buf[7], p_data, len);

    Append_CRC16_Check_Sum(tx_buf, frame_length);

    //HAL_UART_Transmit(&huart1, tx_buf, frame_length, HAL_MAX_DELAY);
}
