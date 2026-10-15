/*====================================================================
<serial_task.cpp>
・シリアル通信まわりのタスク実装

RRST-NHK-Project/ros2can (xiao-esp32-s3_can2io/src/serial_task.cpp) と
同じ52バイト固定長フレーム・状態機械を採用している
(damiao_ctrl/include/damiao_ctrl/frame_codec.hpp のPC側実装と対称)。

Frame Structure:
[START_BYTE][DEVICE_ID][LENGTH][DATA...][CHECKSUM]
- START_BYTE: 0xAA
- DEVICE_ID : config.hppのDEVICE_ID
- LENGTH    : データ部のバイト数 (Tx16NUM * 2 = 48、固定)
- DATA      : int16 x24, ビッグエンディアン
- CHECKSUM  : ID ^ LEN ^ DATA[0]の上位バイト ^ ... ^ DATA[23]の下位バイト
              (START_BYTEは含まない、XOR)
====================================================================*/

#include "serial_task.hpp"
#include "config.hpp"
#include "frame_data.hpp"
#include <Arduino.h>
#include <cstring>

static portMUX_TYPE g_frame_lock = portMUX_INITIALIZER_UNLOCKED;

#define START_BYTE 0xAA

// ================= TX =================

static uint8_t Tx_8Data[1 + 1 + 1 + Tx16NUM * 2 + 1]; // START+ID+LEN+DATA+CHECKSUM

constexpr uint32_t TX_PERIOD_MS = 5; // 送信周期(ros2can他モードと同じ)

// ================= RX =================

enum RxState {
    WAIT_START,
    WAIT_ID,
    WAIT_LEN,
    WAIT_DATA,
    WAIT_CHECKSUM
};

static RxState rx_state = WAIT_START;
static uint8_t rx_id = 0;
static uint8_t rx_len = 0;
static uint8_t rx_buf[Rx16NUM * 2];
static uint8_t rx_index = 0;
static uint8_t rx_checksum = 0;

// ================= TASK =================

void serialTask(void *) {
    TickType_t last_tx = xTaskGetTickCount();

    while (1) {
        receive_frame(); // 毎ループ呼び出す

        if (xTaskGetTickCount() - last_tx >= pdMS_TO_TICKS(TX_PERIOD_MS)) {
            send_frame();
            last_tx = xTaskGetTickCount();
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ================= TX =================

void send_frame() {
    int16_t tx_data_snapshot[Tx16NUM];

    portENTER_CRITICAL(&g_frame_lock);
    std::memcpy(tx_data_snapshot, (const void *)Tx_16Data, sizeof(tx_data_snapshot));
    portEXIT_CRITICAL(&g_frame_lock);

    Tx_8Data[0] = START_BYTE;
    Tx_8Data[1] = DEVICE_ID;
    Tx_8Data[2] = Tx16NUM * 2;

    uint8_t checksum = 0;
    checksum ^= Tx_8Data[1];
    checksum ^= Tx_8Data[2];

    for (int i = 0; i < Tx16NUM; i++) {
        Tx_8Data[3 + i * 2] = (uint8_t)(tx_data_snapshot[i] >> 8);
        Tx_8Data[3 + i * 2 + 1] = (uint8_t)(tx_data_snapshot[i] & 0xFF);
        checksum ^= Tx_8Data[3 + i * 2];
        checksum ^= Tx_8Data[3 + i * 2 + 1];
    }
    Tx_8Data[3 + Tx16NUM * 2] = checksum;

    Serial.write(Tx_8Data, sizeof(Tx_8Data));
}

// ================= RX =================

void receive_frame() {
    while (Serial.available()) {
        uint8_t b = Serial.read();

        switch (rx_state) {
        case WAIT_START:
            if (b == START_BYTE) {
                rx_state = WAIT_ID;
            }
            break;

        case WAIT_ID:
            rx_id = b;
            rx_checksum = b;
            rx_state = WAIT_LEN;
            break;

        case WAIT_LEN:
            rx_len = b;
            rx_checksum ^= b;
            if (rx_len > Rx16NUM * 2) {
                rx_state = WAIT_START; // データ長が不正な場合は同期からやり直し
            } else {
                rx_index = 0;
                rx_state = WAIT_DATA;
            }
            break;

        case WAIT_DATA:
            rx_buf[rx_index++] = b;
            rx_checksum ^= b;
            if (rx_index >= rx_len) {
                rx_state = WAIT_CHECKSUM;
            }
            break;

        case WAIT_CHECKSUM:
            if (rx_checksum == b && rx_id == DEVICE_ID) {
                int16_t decoded_values[Rx16NUM] = {0};
                for (int i = 0; i < rx_len / 2; i++) {
                    decoded_values[i] = (int16_t)((rx_buf[i * 2] << 8) | rx_buf[i * 2 + 1]);
                }
                portENTER_CRITICAL(&g_frame_lock);
                std::memcpy((void *)Rx_16Data, decoded_values, sizeof(decoded_values));
                portEXIT_CRITICAL(&g_frame_lock);
            }
            rx_state = WAIT_START;
            break;
        }
    }
}
