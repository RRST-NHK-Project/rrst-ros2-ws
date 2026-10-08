/*====================================================================
<pin_hunt.cpp>
・CANトランシーバが実際に繋がっているGPIOピンを特定する診断の実装

【この診断の原理】
フェーズ1: 候補GPIOの素のレベルを読む。
  給電された正常なCANトランシーバのRXD出力はプッシュプルで、バスが
  アイドル(recessive)のとき能動的にHIGHを出力する。一方、何も繋がって
  いないピンは浮いてLOWに読める(実測で確認済み)。
  => 「HIGHに読めるピン」= トランシーバのRXDが繋がっているピン。

フェーズ2: 候補の(TX,RX)ペアでTWAIを初期化し、実際に1フレーム送信して
  送信キューが掃けるかを見る。
  - キューが0に掃ける = 本当にワイヤー上へ送信できた(ピンが正しい)
  - キューが詰まったまま = RXがdominant固定でバスアイドルを検出できず、
    一度も送信を開始できていない(ピンが間違っている)
  さらにtx_error_counterを見ると:
  - 掃けた & tx_err増加 = 送信できたが誰もACKしていない(相手が居ない)
  - 掃けた & tx_err=0   = 誰かがACKした = DM3520が応答している!
====================================================================*/

#include "pin_hunt.hpp"
#include "config.hpp"
#include "driver/twai.h"
#include <Arduino.h>

namespace {

// XIAO ESP32-S3で外部に出ているGPIO (D0-D10) + LED。
// D0=1, D1=2, D2=3, D3=4, D4=5, D5=6, D6=43, D7=44, D8=7, D9=8, D10=9
struct PinInfo {
    int gpio;
    const char *label;
};

const PinInfo kCandidatePins[] = {
    {1, "D0"},  {2, "D1"}, {3, "D2"},  {4, "D3"},  {5, "D4"},  {6, "D5"},
    {43, "D6"}, {44, "D7"}, {7, "D8"}, {8, "D9"},  {9, "D10"}, {21, "LED"},
};

struct PinPair {
    int tx;
    int rx;
    const char *label;
};

const PinPair kCandidatePairs[] = {
    {1, 2, "BOARD_SOKI (TX=D0/GPIO1, RX=D1/GPIO2)"},
    {43, 44, "BOARD_MES/SS (TX=D6/GPIO43, RX=D7/GPIO44)"},
    {2, 1, "SOKI逆 (TX=GPIO2, RX=GPIO1)"},
    {44, 43, "MES/SS逆 (TX=GPIO44, RX=GPIO43)"},
};

// 無害な読み取りコマンド(0x7FF宛てのレジスタ読み出し)。モータを動かさない。
void sendProbeFrame() {
    twai_message_t tx{};
    tx.identifier = 0x7FF;
    tx.extd = 0;
    tx.rtr = 0;
    tx.data_length_code = 4;
    tx.data[0] = DAMIAO_MOTOR_ID_1;
    tx.data[1] = 0x00;
    tx.data[2] = 0x33; // read
    tx.data[3] = 0x00; // RID=0
    twai_transmit(&tx, pdMS_TO_TICKS(20));
}

void testPair(const PinPair &pair) {
    Serial.printf("\n[hunt] --- %s ---\n", pair.label);

    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)pair.tx, (gpio_num_t)pair.rx, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[hunt]   driver install 失敗");
        return;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[hunt]   twai_start 失敗");
        twai_driver_uninstall();
        return;
    }
    delay(30);

    sendProbeFrame();

    // 300msかけてキューが掃けるか監視する。
    bool drained = false;
    twai_status_info_t status{};
    for (int i = 0; i < 30; i++) {
        delay(10);
        if (twai_get_status_info(&status) != ESP_OK) {
            continue;
        }
        if (status.msgs_to_tx == 0) {
            drained = true;
            break;
        }
    }
    twai_get_status_info(&status);

    // 受信があれば表示(誰かが応答した証拠)。
    twai_message_t rx;
    int rx_count = 0;
    while (twai_receive(&rx, pdMS_TO_TICKS(5)) == ESP_OK) {
        Serial.printf("[hunt]   *** 受信! id=0x%03lX dlc=%d data=",
                      (unsigned long)rx.identifier, rx.data_length_code);
        for (int i = 0; i < rx.data_length_code; i++) {
            Serial.printf("%02X ", rx.data[i]);
        }
        Serial.println();
        if (++rx_count >= 8) break;
    }

    Serial.printf("[hunt]   送信キュー: %s (msgs_to_tx=%lu)  tx_err=%lu bus_err=%lu 受信=%d件\n",
                  drained ? "掃けた(送信成功)" : "詰まったまま(未送信)",
                  (unsigned long)status.msgs_to_tx, (unsigned long)status.tx_error_counter,
                  (unsigned long)status.bus_error_count, rx_count);

    if (drained && rx_count > 0) {
        Serial.println("[hunt]   ==> このピン組み合わせで通信できています!!");
    } else if (drained && status.tx_error_counter > 0) {
        Serial.println("[hunt]   ==> トランシーバは生きている(送信できた)が、"
                        "ACKを返す相手が居ない");
    } else if (drained) {
        Serial.println("[hunt]   ==> 送信できた(誰かがACKした可能性)");
    } else {
        Serial.println("[hunt]   ==> 未送信。RXがdominant固定でバスアイドルを"
                        "検出できていない(このピンは違う)");
    }

    twai_stop();
    twai_driver_uninstall();
    delay(50);
}

} // namespace

void pinHuntTask(void *) {
    delay(500);
    Serial.println("=================================================");
    Serial.println("[hunt] CANトランシーバ接続ピン特定診断");
    Serial.println("=================================================");

    while (1) {
        // ---- フェーズ1: 全候補ピンの素のレベル ----
        Serial.println("\n[hunt] === フェーズ1: 各GPIOの素のレベル ===");
        Serial.println("[hunt] (給電された正常なトランシーバのRXD = アイドル時HIGH)");
        for (const auto &p : kCandidatePins) {
            pinMode(p.gpio, INPUT);
        }
        delay(50);
        for (const auto &p : kCandidatePins) {
            int high = 0;
            for (int i = 0; i < 20; i++) {
                if (digitalRead(p.gpio) == HIGH) high++;
                delay(2);
            }
            const char *verdict = (high == 20)  ? "HIGH固定 <== トランシーバRXDの可能性大"
                                   : (high == 0) ? "LOW固定"
                                                 : "変動あり <== 信号が来ている可能性";
            Serial.printf("[hunt]   GPIO%-2d (%-3s): HIGH=%2d/20  %s\n", p.gpio, p.label, high,
                          verdict);
        }

        // ---- フェーズ2: 候補ピンペアで実送信テスト ----
        Serial.println("\n[hunt] === フェーズ2: 候補ピンペアでの実送信テスト ===");
        for (const auto &pair : kCandidatePairs) {
            testPair(pair);
        }

        Serial.println("\n[hunt] === 診断完了。10秒後に再実行します ===");
        delay(10000);
    }
}
