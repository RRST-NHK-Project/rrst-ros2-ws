/*====================================================================
<scan_task.cpp>
・DAMIAO_SCAN_MODE専用: CAN ID/ビットレート探索タスクの実装

候補ビットレート(1Mbps -> 500kbps)ごとに、候補CAN ID(0x01-0x7F)へ
Enableコマンド(0xFC)を送り、短時間だけ受信を待ってから念のため
Disableコマンド(0xFD)を送って元に戻す、というのを総当たりで繰り返す。
速度(VEL)・位置(POS_VEL)の指令フレームは一切送らないため、たとえ違う
IDに当たっても回転は発生しない(Enableされるだけで目標値は常に0のまま)。

結果は115200bpsのシリアルモニタ(`pio device monitor -b 115200`)に
テキストで出力される。何か応答があれば、そのときのprobed_id(何番宛てに
送った直後か)とbitrateをconfig.hppのDAMIAO_MOTOR_ID_1/CAN_BITRATE_1MBPS
に反映すること。
====================================================================*/

#include "scan_task.hpp"
#include "config.hpp"
#include <Arduino.h>

namespace {

struct BitrateOption {
    bool is_1mbps;
    const char *label;
};

const BitrateOption kBitrates[] = {
    {true, "1Mbps"},
    {false, "500kbps"},
};

constexpr uint16_t kScanIdMin = 0x01;
constexpr uint16_t kScanIdMax = 0x7F;

bool installDriver(bool is_1mbps) {
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    // TWAI_TIMING_CONFIG_*()マクロはブレース初期化に展開されるため、
    // 三項演算子の中では使えない(コンパイルエラーになる)。if/elseで分岐する。
    twai_timing_config_t t_config;
    if (is_1mbps) {
        t_config = TWAI_TIMING_CONFIG_1MBITS();
    } else {
        t_config = TWAI_TIMING_CONFIG_500KBITS();
    }
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[scan] TWAI install failed");
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[scan] TWAI start failed");
        twai_driver_uninstall();
        return false;
    }
    return true;
}

void uninstallDriver() {
    twai_stop();
    twai_driver_uninstall();
}

void sendControlCmd(uint16_t motor_id, uint8_t cmd) {
    twai_message_t tx{};
    tx.identifier = motor_id;
    tx.extd = 0; // Damiaoは標準11bit ID
    tx.rtr = 0;
    tx.data_length_code = 8;
    for (int i = 0; i < 7; i++) {
        tx.data[i] = 0xFF;
    }
    tx.data[7] = cmd;
    twai_transmit(&tx, pdMS_TO_TICKS(20));
}

// probed_id宛てにコマンドを送った直後、timeout_msだけ受信を待って
// 届いたフレームを全てログに出す(応答元のIDがprobed_idと一致するとは
// 限らない実装もあるため、フィルタせずそのまま出す)。
void drainAndReport(uint16_t probed_id, uint32_t timeout_ms) {
    twai_message_t rx;
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (xTaskGetTickCount() < deadline) {
        if (twai_receive(&rx, pdMS_TO_TICKS(2)) == ESP_OK) {
            Serial.printf(
                "[scan]   *** response! probed_id=0x%02X rx_id=0x%03X ext=%d dlc=%d data=",
                probed_id, rx.identifier, rx.extd, rx.data_length_code);
            for (int i = 0; i < rx.data_length_code; i++) {
                Serial.printf("%02X ", rx.data[i]);
            }
            Serial.println();
        }
    }
}

} // namespace

void scanTask(void *) {
    delay(500);
    Serial.println("=================================================");
    Serial.println("[scan] Damiao CAN ID / bitrate scan start");
    Serial.println("[scan] Enable/Disable only. No motion command is sent.");
    Serial.println("=================================================");

    for (const auto &br : kBitrates) {
        Serial.printf("\n[scan] ---- trying %s ----\n", br.label);
        if (!installDriver(br.is_1mbps)) {
            continue;
        }
        delay(50); // トランシーバ安定待ち

        for (uint16_t id = kScanIdMin; id <= kScanIdMax; id++) {
            sendControlCmd(id, 0xFC); // Enable
            drainAndReport(id, 15);
            sendControlCmd(id, 0xFD); // Disable(念のため元に戻す)
            drainAndReport(id, 5);
        }

        // 配線診断用: バスの状態・エラーカウンタを出力する。
        // - state=RUNNINGでtx_error_counter/bus_error_countが0付近のまま
        //   ずっと増えない場合、送信自体がACKもエラーも返ってこない
        //   (=物理的にCAN-H/CAN-Lがどこにも繋がっていない、または
        //   トランシーバが動作していない)可能性が高い。
        // - tx_error_counterが増え続けてBUS_OFFに落ちている場合は、
        //   配線はされているがACKが返らない(相手が居ない/ビットレート
        //   不一致/H-L逆)ことを示唆する。
        twai_status_info_t status{};
        if (twai_get_status_info(&status) == ESP_OK) {
            const char *state_str =
                status.state == TWAI_STATE_RUNNING     ? "RUNNING"
                : status.state == TWAI_STATE_BUS_OFF   ? "BUS_OFF"
                : status.state == TWAI_STATE_STOPPED   ? "STOPPED"
                : status.state == TWAI_STATE_RECOVERING ? "RECOVERING"
                                                         : "UNKNOWN";
            Serial.printf(
                "[scan] bus status: state=%s tx_err=%lu rx_err=%lu bus_err_count=%lu "
                "tx_failed=%lu arb_lost=%lu\n",
                state_str, (unsigned long)status.tx_error_counter,
                (unsigned long)status.rx_error_counter, (unsigned long)status.bus_error_count,
                (unsigned long)status.tx_failed_count, (unsigned long)status.arb_lost_count);
        } else {
            Serial.println("[scan] twai_get_status_info failed");
        }

        uninstallDriver();
    }

    Serial.println("\n[scan] scan complete.");
    Serial.println("[scan] Nothing found above? check the following:");
    Serial.println("[scan]  - CAN-H/CAN-L wiring and termination resistor(TERM)");
    Serial.println("[scan]  - DM3520 power is actually on (separate from ESP32 5V/USB power)");
    Serial.println("[scan]  - motor CAN ID might be outside the 0x01-0x7F range scanned here");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
