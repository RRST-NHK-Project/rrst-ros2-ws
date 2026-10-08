/*====================================================================
<scan_task.cpp>
・DAMIAO_SCAN_MODE専用: CAN ID/ビットレート探索タスクの実装

候補ビットレート(1Mbps -> 500kbps)ごとに、候補CAN ID(0x01-0x7F)へ
Enableコマンド(0xFC)を送り、短時間だけ受信を待ってから念のため
Disableコマンド(0xFD)を送って元に戻す、というのを総当たりで繰り返す。
速度(VEL)・位置(POS_VEL)の指令フレームは一切送らないため、たとえ違う
IDに当たっても回転は発生しない(Enableされるだけで目標値は常に0のまま)。

結果は115200bpsのシリアルモニタ(pio device monitorや自作capture_serial.py)
にテキストで出力される。何か応答があれば、そのときのprobed_id(何番宛てに
送った直後か)とbitrateをconfig.hppのDAMIAO_MOTOR_ID_1/CAN_BITRATE_1MBPS
に反映すること。

配線診断の実績(2026-09-28のデバッグ経緯):
- twai_transmit()の戻り値を見ていなかった際、tx_error_counterが常に0の
  まま何も送受信できない現象が続いた。
- 戻り値を集計したところ ok=6 fail=248 (ESP_ERR_TIMEOUT) で、TXキューが
  6個で恒久的に詰まっていることが判明(既定のtx_queue_len+実際に送信を
  試みている1フレーム分で6)。
- TWAI_MODE_NO_ACK(外部ACK不要の自己テストモード)でも同じ結果になった。
  ただしNO_ACKモードは「バスアイドル検出」自体は省略しないため、これは
  ハードウェア無関係の証明にはならない(要注意、誤った結論を出しかけた)。
- TWAIドライバを介さない生のdigitalRead()でCAN_RXピンを直接読んだところ、
  常時LOW(dominant固定)と判明。これがバスアイドルを検出できず一切送信を
  開始できない根本原因だった。
====================================================================*/

#include "scan_task.hpp"
#include "config.hpp"
#include <Arduino.h>

namespace {

enum class Bitrate { k1M, k500K, k250K, k125K };

struct BitrateOption {
    Bitrate rate;
    const char *label;
};

const BitrateOption kBitrates[] = {
    {Bitrate::k1M, "1Mbps"},
    {Bitrate::k500K, "500kbps"},
    {Bitrate::k250K, "250kbps"},
    {Bitrate::k125K, "125kbps"},
};

constexpr uint16_t kScanIdMin = 0x01;
constexpr uint16_t kScanIdMax = 0x7F;

bool installDriver(Bitrate rate, twai_mode_t mode = TWAI_MODE_NORMAL) {
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, mode);
    // TWAI_TIMING_CONFIG_*()マクロはブレース初期化に展開されるため、
    // 三項演算子の中では使えない(コンパイルエラーになる)。if/elseで分岐する。
    twai_timing_config_t t_config;
    if (rate == Bitrate::k1M) {
        t_config = TWAI_TIMING_CONFIG_1MBITS();
    } else if (rate == Bitrate::k500K) {
        t_config = TWAI_TIMING_CONFIG_500KBITS();
    } else if (rate == Bitrate::k250K) {
        t_config = TWAI_TIMING_CONFIG_250KBITS();
    } else {
        t_config = TWAI_TIMING_CONFIG_125KBITS();
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

// 戻り値をここまでチェックしていなかったため、送信自体が毎回失敗していても
// 気づけない状態だった。呼び出し側で成功/失敗を集計できるようesp_err_tを返す。
esp_err_t sendControlCmd(uint16_t motor_id, uint8_t cmd) {
    twai_message_t tx{};
    tx.identifier = motor_id;
    tx.extd = 0; // Damiaoは標準11bit ID
    tx.rtr = 0;
    tx.data_length_code = 8;
    for (int i = 0; i < 7; i++) {
        tx.data[i] = 0xFF;
    }
    tx.data[7] = cmd;
    return twai_transmit(&tx, pdMS_TO_TICKS(20));
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

// TWAIドライバを一切使わず、CAN_RX(GPIO)の生の電圧レベルを直接読む。
// TWAIドライバがピンをGPIOマトリクス経由でCANコントローラに繋ぎ変える前の、
// 最も単純な電気的な状態を確認するための切り分け用。
// 指定ピンをINPUTにして500ms間サンプリングし、HIGH/LOW回数を返す。
void sampleDigitalLevel(int pin, int &high_count, int &low_count) {
    pinMode(pin, INPUT);
    high_count = 0;
    low_count = 0;
    for (int i = 0; i < 50; i++) {
        (digitalRead(pin) == HIGH ? high_count : low_count)++;
        delay(10);
    }
}

void rawGpioLevelCheck() {
    Serial.println("\n[scan] ---- raw GPIO level check (TWAI未使用) ----");

    int rx_high, rx_low;
    sampleDigitalLevel(CAN_RX, rx_high, rx_low);
    Serial.printf(
        "[scan] CAN_RX(GPIO%d) raw level over 500ms: HIGH=%d LOW=%d\n", CAN_RX, rx_high, rx_low);

    int tx_high, tx_low;
    sampleDigitalLevel(CAN_TX, tx_high, tx_low);
    Serial.printf(
        "[scan] CAN_TX(GPIO%d) raw level over 500ms: HIGH=%d LOW=%d\n", CAN_TX, tx_high, tx_low);

    // 比較基準: 何も接続されていないはずの予備ピン(GPIO4, GPIO5)を同条件で
    // 読む。これが同じく常時LOWになるなら、CAN_RX固有の問題ではなく
    // 「このボードのfloating GPIOは何もしなくてもLOWに読める」というだけの
    // 話になる(内部プルダウンがデフォルト等)。
    int ref1_high, ref1_low, ref2_high, ref2_low;
    sampleDigitalLevel(4, ref1_high, ref1_low);
    sampleDigitalLevel(5, ref2_high, ref2_low);
    Serial.printf(
        "[scan] (参考)GPIO4 raw level: HIGH=%d LOW=%d / GPIO5 raw level: HIGH=%d LOW=%d\n",
        ref1_high, ref1_low, ref2_high, ref2_low);

    if (rx_low == 50 && (ref1_low == 50 || ref2_low == 50)) {
        Serial.println(
            "[scan] -> CAN_RXだけでなく未接続の予備ピンも常時LOW。CAN_RX固有の"
            "問題ではなく、このボードのfloating GPIOはデフォルトでLOWに読める"
            "だけの可能性が高い(このテスト自体は配線診断として無効)。");
    } else if (rx_low == 50) {
        Serial.println(
            "[scan] -> CAN_RXだけが常時LOW(予備ピンはHIGH/変動)。CAN_RX固有の"
            "電気的な問題(GNDショート・断線・トランシーバ未給電等)の疑いが強い。");
    } else if (rx_high == 50) {
        Serial.println("[scan] -> CAN_RXは常時HIGH(recessive/アイドル)。TWAI層の問題を疑う。");
    } else {
        Serial.println("[scan] -> CAN_RXは変動あり(ノイズ拾い等の可能性)。");
    }
}

// TWAI_MODE_NO_ACK(自己テストモード): 外部からのACKを一切必要とせず、
// 自分の送信を自分で完了扱いにできる特殊モード。
// 注意: このモードは「送信開始前のバスアイドル検出」までは省略しない。
// そのためこのテストが失敗しても、ハードウェア無関係とは断定できない
// (rawGpioLevelCheck()の結果と合わせて判断すること)。
void runSelfTest() {
    Serial.println("\n[scan] ---- self-test (TWAI_MODE_NO_ACK) ----");
    if (!installDriver(Bitrate::k1M, TWAI_MODE_NO_ACK)) {
        Serial.println("[scan] self-test: driver install failed");
        return;
    }
    delay(50);

    int ok = 0, fail = 0;
    esp_err_t last_err = ESP_OK;
    for (int i = 0; i < 20; i++) {
        esp_err_t r = sendControlCmd(0x01, 0xFC);
        (r == ESP_OK ? ok : fail)++;
        if (r != ESP_OK) last_err = r;
        delay(5);
    }
    twai_status_info_t status{};
    twai_get_status_info(&status);
    Serial.printf(
        "[scan] self-test result: ok=%d fail=%d (last_err=%s) msgs_to_tx=%lu tx_err=%lu\n",
        ok, fail, esp_err_to_name(last_err), (unsigned long)status.msgs_to_tx,
        (unsigned long)status.tx_error_counter);

    uninstallDriver();
    delay(50);
}

} // namespace

void scanTask(void *) {
    delay(500);
    Serial.println("=================================================");
    Serial.println("[scan] Damiao CAN ID / bitrate scan start");
    Serial.println("[scan] Enable/Disable only. No motion command is sent.");
    Serial.println("=================================================");

    rawGpioLevelCheck();
    runSelfTest();

    // 公式STM32サンプル(dmBots/motor-control-routine)の発見を踏まえた
    // 重点テスト: Enable/DisableコマンドのCAN IDはmotor_idそのものではなく
    // 「motor_id + モードオフセット」である可能性が高い
    // (MIT=+0x000, POS=+0x100, SPD=+0x200, PSI=+0x300)。
    // これまでのスキャンは全てオフセット無し(MIT相当)でしか試していな
    // かったため、本命のmotor_id=0x01に対して全オフセット×全ビットレート
    // を先に重点的に試す。
    Serial.println("\n[scan] ==== 重点テスト: motor_id=0x01 x 全モードオフセット ====");
    {
        constexpr uint16_t kCandidateMotorId = 0x01;
        constexpr uint16_t kModeOffsets[] = {0x000, 0x100, 0x200, 0x300};
        constexpr const char *kModeNames[] = {"MIT(0x000)", "POS(0x100)", "SPD(0x200)", "PSI(0x300)"};

        for (const auto &br : kBitrates) {
            if (!installDriver(br.rate)) {
                continue;
            }
            delay(30);
            for (int m = 0; m < 4; m++) {
                const uint16_t id = kCandidateMotorId + kModeOffsets[m];
                Serial.printf("[scan] %s bitrate, id=0x%03X (%s) へEnable送信...\n", br.label, id,
                              kModeNames[m]);
                sendControlCmd(id, 0xFC);
                drainAndReport(id, 100);
                sendControlCmd(id, 0xFD);
                drainAndReport(id, 20);
            }
            uninstallDriver();
            delay(20);
        }
    }
    Serial.println("[scan] ==== 重点テスト完了 ====\n");

    // 拡張テスト: motor_id=0x01だけでは応答が無かったため、全motor_id候補
    // (0x01-0x7F)×全モードオフセット(MIT/POS/SPD/PSI)を1Mbpsに絞って
    // 総当たりする。4ビットレート分をこの粒度でやると時間がかかりすぎる
    // ため、まずは最も一般的な1Mbpsのみ。
    Serial.println("\n[scan] ==== 拡張テスト: 全motor_id x 全モードオフセット (1Mbps) ====");
    {
        constexpr uint16_t kModeOffsets[] = {0x000, 0x100, 0x200, 0x300};
        if (installDriver(Bitrate::k1M)) {
            delay(30);
            int resets = 0;
            for (uint16_t base_id = kScanIdMin; base_id <= kScanIdMax; base_id++) {
                for (int m = 0; m < 4; m++) {
                    const uint16_t id = base_id + kModeOffsets[m];
                    esp_err_t r1 = sendControlCmd(id, 0xFC);
                    if (r1 == ESP_ERR_TIMEOUT) {
                        uninstallDriver();
                        installDriver(Bitrate::k1M);
                        delay(5);
                        r1 = sendControlCmd(id, 0xFC);
                        resets++;
                    }
                    drainAndReport(id, 8);
                    esp_err_t r2 = sendControlCmd(id, 0xFD);
                    if (r2 == ESP_ERR_TIMEOUT) {
                        uninstallDriver();
                        installDriver(Bitrate::k1M);
                        delay(5);
                        r2 = sendControlCmd(id, 0xFD);
                        resets++;
                    }
                    drainAndReport(id, 3);
                }
            }
            Serial.printf("[scan] extended test driver resets: %d\n", resets);
            uninstallDriver();
        }
    }
    Serial.println("[scan] ==== 拡張テスト完了 ====\n");

    // 公式STM32サンプル(dm_motor_drv.c)のread_motor_data()と同じ、
    // レジスタ読み取りコマンド。固定CAN ID 0x7FFへ、問い合わせ対象の
    // motor_idをデータ部に埋め込んで送る方式。motor_id自体が分からなくても
    // 0x7FF宛ての送信は一度で済むが、対象を変えるにはデータを変えて
    // 複数回送る必要がある。RID=0(UV_VALUE)を全motor_id候補に対して問い合わせる。
    Serial.println("\n[scan] ==== 0x7FFレジスタ読み取りテスト (1Mbps) ====");
    {
        if (installDriver(Bitrate::k1M)) {
            delay(30);
            for (uint16_t id = kScanIdMin; id <= kScanIdMax; id++) {
                uint8_t can_id_l = id & 0xFF;
                uint8_t can_id_h = (id >> 8) & 0x07;
                uint8_t data[4] = {can_id_l, can_id_h, 0x33, 0x00}; // RID=0 (UV_VALUE)
                twai_message_t tx{};
                tx.identifier = 0x7FF;
                tx.extd = 0;
                tx.rtr = 0;
                tx.data_length_code = 4;
                memcpy(tx.data, data, 4);
                esp_err_t r = twai_transmit(&tx, pdMS_TO_TICKS(20));
                if (r == ESP_ERR_TIMEOUT) {
                    uninstallDriver();
                    installDriver(Bitrate::k1M);
                    delay(5);
                }
                drainAndReport(id, 10);
            }
            uninstallDriver();
        }
    }
    Serial.println("[scan] ==== 0x7FFテスト完了 ====\n");

    for (const auto &br : kBitrates) {
        Serial.printf("\n[scan] ---- trying %s ----\n", br.label);
        if (!installDriver(br.rate)) {
            continue;
        }
        delay(50); // トランシーバ安定待ち

        // 受信専用の待機(こちらからは何も送らない)。DM3520が自発的に
        // 何かを送信している場合、ここで直接捕まえられる。
        Serial.println("[scan] passive listen (800ms, 送信なし)...");
        {
            twai_message_t rx;
            const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(800);
            int count = 0;
            while (xTaskGetTickCount() < deadline) {
                if (twai_receive(&rx, pdMS_TO_TICKS(20)) == ESP_OK) {
                    count++;
                    Serial.printf(
                        "[scan]   *** passive RX! id=0x%03lX ext=%d dlc=%d data=",
                        (unsigned long)rx.identifier, rx.extd, rx.data_length_code);
                    for (int i = 0; i < rx.data_length_code; i++) {
                        Serial.printf("%02X ", rx.data[i]);
                    }
                    Serial.println();
                }
            }
            Serial.printf("[scan] passive listen: %d frame(s) received\n", count);
        }

        // 既知のESP-IDF TWAIドライバのバグ(espressif/esp-idf#19068)対策:
        // TXパイプラインがレースコンディションで永久にスタックすることがある
        // (エラーカウンタは0のまま、bus-offにもならず、キューが詰まった
        // フレームは二度と送信されない)。ESP_ERR_TIMEOUTを検知したら
        // ドライバを再インストールして復旧する。これをしないと、最初の
        // 数回の送信でスタックし、残り全IDが一度も実際には送信されない
        // (実測: 最初の3ID=6送信のみ成功し、以降124ID分は未送信のまま
        // "失敗"していた)。
        int tx_ok = 0, tx_fail = 0, resets = 0;
        esp_err_t last_fail_err = ESP_OK;
        for (uint16_t id = kScanIdMin; id <= kScanIdMax; id++) {
            esp_err_t r1 = sendControlCmd(id, 0xFC); // Enable
            if (r1 == ESP_ERR_TIMEOUT) {
                uninstallDriver();
                installDriver(br.rate);
                delay(10);
                r1 = sendControlCmd(id, 0xFC);
                resets++;
            }
            (r1 == ESP_OK ? tx_ok : tx_fail)++;
            if (r1 != ESP_OK) last_fail_err = r1;
            drainAndReport(id, 15);

            esp_err_t r2 = sendControlCmd(id, 0xFD); // Disable(念のため元に戻す)
            if (r2 == ESP_ERR_TIMEOUT) {
                uninstallDriver();
                installDriver(br.rate);
                delay(10);
                r2 = sendControlCmd(id, 0xFD);
                resets++;
            }
            (r2 == ESP_OK ? tx_ok : tx_fail)++;
            if (r2 != ESP_OK) last_fail_err = r2;
            drainAndReport(id, 5);
        }
        Serial.printf("[scan] driver resets needed: %d\n", resets);
        Serial.printf(
            "[scan] twai_transmit results: ok=%d fail=%d (last_fail_err=%s)\n",
            tx_ok, tx_fail, esp_err_to_name(last_fail_err));

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
                "tx_failed=%lu arb_lost=%lu msgs_to_tx=%lu msgs_to_rx=%lu\n",
                state_str, (unsigned long)status.tx_error_counter,
                (unsigned long)status.rx_error_counter, (unsigned long)status.bus_error_count,
                (unsigned long)status.tx_failed_count, (unsigned long)status.arb_lost_count,
                (unsigned long)status.msgs_to_tx, (unsigned long)status.msgs_to_rx);
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
