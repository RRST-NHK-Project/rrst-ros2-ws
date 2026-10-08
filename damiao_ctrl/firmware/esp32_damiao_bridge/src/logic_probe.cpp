/*====================================================================
<logic_probe.cpp>
・簡易ロジックアナライザの実装(送信機能付き)

CAN_RXピンの電圧変化(HIGH<->LOW遷移)をGPIO割り込みで検出し、遷移の
タイミング(マイクロ秒単位)をリングバッファに記録する。

2026-10-01: 単に受信を待つだけでは何も捉えられなかったため、TWAI経由で
実際にEnableシーケンス(Disable->ClearError->モードレジスタ書き込み->
Enable->POS_VEL)を送信しながら、同時にCAN_RXの生電圧遷移を記録する
モードを追加した。ESP32のGPIO入力はピンに複数の内部信号(TWAI受信用と
plain digitalRead割り込みの両方)を同時に接続できるため、TWAIで送信しつつ
素のGPIOレベルも監視できる。
====================================================================*/

#include "logic_probe.hpp"
#include "config.hpp"
#include "driver/twai.h"
#include <Arduino.h>
#include <cstring>

namespace {

constexpr int BUF_SIZE = 8000;
volatile uint32_t g_timestamps_us[BUF_SIZE];
volatile uint8_t g_levels[BUF_SIZE];
volatile int g_count = 0;

void IRAM_ATTR onEdge() {
    if (g_count >= BUF_SIZE) {
        return;
    }
    g_timestamps_us[g_count] = micros();
    g_levels[g_count] = (uint8_t)digitalRead(CAN_RX);
    g_count++;
}

void dumpCapture() {
    const int n = g_count;
    Serial.printf("[probe] 記録終了: %d 回の電圧遷移を検出\n", n);

    if (n == 0) {
        Serial.println("[probe] -> 電圧変化が一切ありませんでした(完全に静止)。");
        return;
    }

    uint32_t min_delta = 0xFFFFFFFF, max_delta = 0;
    uint64_t sum_delta = 0;
    uint32_t prev_t = g_timestamps_us[0];
    for (int i = 1; i < n; i++) {
        uint32_t delta = g_timestamps_us[i] - prev_t;
        prev_t = g_timestamps_us[i];
        if (delta < min_delta) min_delta = delta;
        if (delta > max_delta) max_delta = delta;
        sum_delta += delta;
    }
    if (n > 1) {
        Serial.printf(
            "[probe] delta統計(us): min=%lu max=%lu avg=%.1f (n=%d)\n",
            (unsigned long)min_delta, (unsigned long)max_delta, (double)sum_delta / (n - 1),
            n - 1);
    }

    Serial.println("[probe] 先頭80件の遷移 (経過us, レベル):");
    uint32_t t0 = g_timestamps_us[0];
    for (int i = 0; i < n && i < 80; i++) {
        Serial.printf(
            "  t=%8lu us  level=%s\n", (unsigned long)(g_timestamps_us[i] - t0),
            g_levels[i] == HIGH ? "HIGH" : "LOW");
    }
    if (n >= BUF_SIZE) {
        Serial.println("[probe] -> バッファ満杯(高速/頻繁な遷移)。");
    }
}

// ---- TWAI送信用の最小ヘルパー(damiao.cppと同じプロトコル) ----

void sendStandard(uint32_t can_id, const uint8_t *data, uint8_t len) {
    twai_message_t tx{};
    tx.identifier = can_id;
    tx.extd = 0;
    tx.rtr = 0;
    tx.data_length_code = len;
    for (uint8_t i = 0; i < len; i++) {
        tx.data[i] = data[i];
    }
    twai_transmit(&tx, pdMS_TO_TICKS(20));
}

void sendControlCmd(uint16_t motor_can_id, uint8_t cmd) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
    sendStandard(motor_can_id, data, 8);
}

void writeRegister(uint16_t motor_can_id, uint8_t rid, int32_t value) {
    uint8_t can_id_l = motor_can_id & 0xFF;
    uint8_t can_id_h = (motor_can_id >> 8) & 0x07;
    uint8_t data[8] = {can_id_l, can_id_h, 0x55, rid, 0, 0, 0, 0};
    std::memcpy(&data[4], &value, 4);
    sendStandard(0x7FF, data, 8);
}

void sendPosVel(uint16_t motor_can_id, float pos_rad, float vel_radps) {
    uint8_t data[8];
    std::memcpy(data, &pos_rad, 4);
    std::memcpy(data + 4, &vel_radps, 4);
    sendStandard(0x100 + motor_can_id, data, 8);
}

bool installTwai() {
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        return false;
    }
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }
    return true;
}

} // namespace

void logicProbeTask(void *) {
    delay(500);
    Serial.println("=================================================");
    Serial.println("[probe] 簡易ロジックアナライザ (送信+生電圧記録)");
    Serial.printf("[probe] CAN_RX(GPIO%d)の電圧遷移を記録します\n", CAN_RX);
    Serial.println("=================================================");

    while (1) {
        pinMode(CAN_RX, INPUT);
        g_count = 0;

        if (!installTwai()) {
            Serial.println("[probe] TWAI install failed");
            delay(3000);
            continue;
        }
        delay(30);

        attachInterrupt(digitalPinToInterrupt(CAN_RX), onEdge, CHANGE);

        Serial.printf("\n[probe] Enableシーケンス送信開始 (motor_id=0x%02X)...\n",
                      DAMIAO_MOTOR_ID_1);
        sendControlCmd(DAMIAO_MOTOR_ID_1, 0xFD); // Disable
        delay(60);
        sendControlCmd(DAMIAO_MOTOR_ID_1, 0xFB); // ClearError
        delay(60);
        writeRegister(DAMIAO_MOTOR_ID_1, 10, 2); // CMODE = POS_VEL
        delay(60);
        sendControlCmd(DAMIAO_MOTOR_ID_1, 0xFC); // Enable
        delay(60);
        for (int i = 0; i < 20; i++) {
            sendPosVel(DAMIAO_MOTOR_ID_1, 0.0f, 0.3f);
            delay(20);
        }

        detachInterrupt(digitalPinToInterrupt(CAN_RX));
        twai_stop();
        twai_driver_uninstall();

        dumpCapture();

        Serial.println("[probe] 3秒後に再送信します...");
        delay(3000);
    }
}
