/*====================================================================
<cubemars.cpp>
・MODE_CUBEMARS(CubeMars AKシリーズ Servo(CAN)モード ドライバ)の実装ファイル

CubeMars AK Series Module Product Manual V3.2.0 4.1節の Servo(CAN)モードプロトコル
(拡張CAN ID = 制御モードID(bit28-8) + ドライバID(bit7-0)、1Mbps固定)を直接喋る。
AK40-10等のAKシリーズはアクチュエータ内蔵のクローズドループ(位置/速度)が指令に
そのまま追従するため、ロボマスのGM6020のようなホスト側PID(PID.hpp)は不要。

速度/位置ループに加え、マニュアル4.2節のForce Control(MIT)モード
(control_mode_id=8、位置+速度+Kp+Kd+トルクFFを1フレームで指令するインピーダンス
制御)にも対応する。CAN IDやフレーム形式はServo(CAN)モードと同じ拡張ID方式なので、
本機はcontrol_modeスロットの値でどちらのモードを使うか毎周期選択するだけでよい。

スロット割り当て (frame_data.hppのTx_16Data/Rx_16Dataを使用、独立デバイスとして
24スロットをそのまま使う。ノード/スロット分配は行わない):

  Rx_16Data (PC -> 本機, 指令):
    0-3: target (モータ1-4、意味はcontrol_modeに依存)
         - control_mode=0(速度): 電気角速度、10ERPM/LSB (レンジ ±327670ERPM)
           マニュアル4.1.4の帰還フレーム速度フィールドと同一スケール
         - control_mode=1(位置): 0.1deg/LSB (レンジ ±3276.7deg)
           マニュアル4.3.1の帰還フレーム位置フィールドと同一スケール
         - control_mode=2(MIT): 目標位置、0.1deg/LSB (position/positionループと同一
           スケール。CAN送信直前にradへ変換する)
    4-7: control_mode (モータ1-4): 0=速度ループ, 1=位置ループ, 2=MIT(Force Control),
         3=Set Origin(原点設定、マニュアル4.1節。target[m]をorigin_modeとして
         流用: 0=一時原点/1=永久原点(フラッシュ保存)/2=デフォルト原点へ復元。
         フラッシュ書き込みを避けるためエッジ検出で1回だけ送信し、以後は
         mode=3が継続していても再送しない。詳細はcubemars.cpp本体のコメント参照)
         全ゼロ(E-STOP/未接続時のデフォルト)で0=速度・target=0となり、
         安全にゼロ速度指令(その場停止)になる設計。位置ジャンプは発生しない。
    8-11:  MITモード用 目標速度(モータ1-4)、0.01rad/s/LSB (control_mode=2のみ参照)
    12-15: MITモード用 Kp(モータ1-4)、0.1/LSB、レンジ0-500 (control_mode=2のみ参照)
    16-19: MITモード用 Kd(モータ1-4)、0.01/LSB、レンジ0-5 (control_mode=2のみ参照)
    20-23: MITモード用 目標トルクFF(モータ1-4)、0.01N・m/LSB (control_mode=2のみ参照)

  Tx_16Data (本機 -> PC, 帰還。マニュアル4.3.1 CAN Upload Message Protocol
             (Function ID 0x29) の値をスケール変換無しでそのまま格納):
    0-3: position [0.1deg/LSB]
    4-7: speed    [10ERPM/LSB] (電気角速度。出力軸rpmへの換算はGUIプロファイルの
                                 scale項目で行う想定、極対数・減速比はモータ機種依存のため)
    8-11: current [0.01A/LSB]
    12-15: motor temperature [degC]
    16-19: error code (0=no fault, 1=motor over-temp, 2=over-current, 3=over-voltage,
                        4=under-voltage, 5=encoder fault, 6=MOSFET over-temp, 7=motor stall)
    20-23: 帰還フレーム受信カウンタ [cubemars_ctrlで追加。元は未使用]
           0x29フレームを受信するたびに+1(int16で折り返す)。ROS側はこの値が
           変化しているかでモータの応答有無を判定する。

Copyright (c) 2025 RRST-NHK-Project
SPDX-License-Identifier: MIT
====================================================================*/

#include "cubemars.hpp"
#include "config.hpp"
#include "defs.hpp"
#include "frame_data.hpp"
#include "serial_task.hpp"
#include <Arduino.h>
#include <algorithm>

namespace {

// Servo(CAN)モードの制御モードID (マニュアル4.1節)
constexpr uint32_t CUBEMARS_CMD_RPM = 3; // Velocity Loop Mode
constexpr uint32_t CUBEMARS_CMD_POS = 4; // Position Loop Mode
// Set Origin Mode (マニュアル4.1節。RPM=3/POS=4と同じ制御モードID体系の続き番号。
// 実機のR-Link/マニュアル記載と一致するか要確認 [2026-08-27時点未検証]。
// data[0](1byte): 0=一時原点(電源off/onで消える) / 1=永久原点(フラッシュ保存)
// / 2=デフォルト原点へ復元)
constexpr uint32_t CUBEMARS_CMD_SET_ORIGIN = 5;
// Force Control(MIT)モードの制御モードID (マニュアル4.2節、Servo(CAN)モードと
// 同じ拡張ID方式(control_mode_id<<8 | driver_id)を共有する)
constexpr uint32_t CUBEMARS_CMD_MIT = 8;
constexpr uint32_t CUBEMARS_FEEDBACK_FUNCTION_ID = 0x29; // 定期帰還フレーム

// PC側(Rx_16Data)のcontrol_mode enum値
constexpr int16_t CUBEMARS_MODE_VELOCITY = 0;
constexpr int16_t CUBEMARS_MODE_POSITION = 1;
constexpr int16_t CUBEMARS_MODE_MIT = 2;
constexpr int16_t CUBEMARS_MODE_SET_ORIGIN = 3;

// MITモード用の追加スロット (target/control_modeは0-7を流用、cubemars.cpp先頭コメント参照)
constexpr int MIT_SLOT_VELOCITY = 8;   // 8-11
constexpr int MIT_SLOT_KP = 12;        // 12-15
constexpr int MIT_SLOT_KD = 16;        // 16-19
constexpr int MIT_SLOT_TORQUE_FF = 20; // 20-23

// MITモード各スロットのスケール (Ros2CanCubemarsPacketController.hppと一致させること)
constexpr double MIT_POSITION_LSB_DEG = 0.1;    // 目標位置(target流用)。0.1deg/LSB
constexpr double MIT_VELOCITY_LSB_RADPS = 0.01; // 目標速度。0.01rad/s/LSB
constexpr double MIT_KP_LSB = 0.1;              // Kp。0.1/LSB
constexpr double MIT_KD_LSB = 0.01;             // Kd。0.01/LSB
constexpr double MIT_TORQUE_LSB_NM = 0.01;      // トルクFF。0.01N・m/LSB

const uint8_t kMotorCanId[CUBEMARS_MOTOR_COUNT] = {
    CUBEMARS_MOTOR_ID_1,
#if CUBEMARS_MOTOR_COUNT > 1
    CUBEMARS_MOTOR_ID_2,
#endif
#if CUBEMARS_MOTOR_COUNT > 2
    CUBEMARS_MOTOR_ID_3,
#endif
#if CUBEMARS_MOTOR_COUNT > 3
    CUBEMARS_MOTOR_ID_4,
#endif
};

// CAN未接続/ESC無応答が続くとエラーカウンタが上限に達しBus-Offへ遷移するが、
// TWAIドライバはBus-Offから自動復帰しない(twai_initiate_recovery()が必須、かつ
// 復帰後もRUNNINGへは戻らずSTOPPEDで止まるため明示的にtwai_start()が要る)。
// can_task.cppのcanRecoverBusIfNeeded()と同じ対策(詳細はそちらのコメント参照)。
void recoverBusIfNeeded() {
    constexpr uint32_t CAN_RECOVERY_CHECK_PERIOD_MS = 100;
    static uint32_t last_check_ms = 0;
    const uint32_t now_ms = millis();
    if (now_ms - last_check_ms < CAN_RECOVERY_CHECK_PERIOD_MS) {
        return;
    }
    last_check_ms = now_ms;

    twai_status_info_t status{};
    if (twai_get_status_info(&status) != ESP_OK) {
        return;
    }

    if (status.state == TWAI_STATE_BUS_OFF) {
        twai_initiate_recovery();
    } else if (status.state == TWAI_STATE_STOPPED) {
        twai_start();
    }
}

// twai_transmit失敗ログはserialTaskのバイナリフレームと同じUARTに出るため、
// 失敗し続ける間(200Hzループ)毎回出すとテキストが混入してホスト側のフレーム
// 同期が壊れる。間隔を絞って出す。
void logTransmitFailure(const char *message) {
    constexpr uint32_t LOG_THROTTLE_MS = 500;
    static uint32_t last_log_ms = 0;
    const uint32_t now_ms = millis();
    if (now_ms - last_log_ms < LOG_THROTTLE_MS) {
        return;
    }
    last_log_ms = now_ms;
    Serial.println(message);
}

void sendExtended(uint32_t cmd_id, uint8_t motor_can_id, const uint8_t *data, uint8_t len) {
    twai_message_t tx{};
    tx.identifier = (cmd_id << 8) | motor_can_id;
    tx.extd = 1;
    tx.rtr = 0;
    tx.data_length_code = len;
    for (uint8_t i = 0; i < len; i++) {
        tx.data[i] = data[i];
    }
    if (twai_transmit(&tx, pdMS_TO_TICKS(20)) != ESP_OK) {
        logTransmitFailure("[ERR] cubemars: twai_transmit failed");
    }
}

// Set Originコマンド送信済みフラグ(モータごと)。mode=3が複数周期にわたって
// 継続送信されても、フラッシュ書き込みを伴う実コマンドは1回だけ送るための
// エッジ検出用(sendCommands()参照)。
bool g_origin_cmd_sent[CUBEMARS_MOTOR_COUNT] = {false};

void sendSetOriginCommand(uint8_t motor_can_id, uint8_t origin_mode) {
    uint8_t buffer[1] = {origin_mode};
    sendExtended(CUBEMARS_CMD_SET_ORIGIN, motor_can_id, buffer, 1);
}

void sendInt32Command(uint32_t cmd_id, uint8_t motor_can_id, int32_t value) {
    uint8_t buffer[4];
    buffer[0] = (uint8_t)(value >> 24);
    buffer[1] = (uint8_t)(value >> 16);
    buffer[2] = (uint8_t)(value >> 8);
    buffer[3] = (uint8_t)value;
    sendExtended(cmd_id, motor_can_id, buffer, 4);
}

// float値をx_min~x_maxの範囲へクランプしたうえでbitsビットの符号無し整数へ
// エンコードする (マニュアル4.2節 float_to_uint() と同じ変換式)。
// x_min/x_maxはモータ側のデコード基準と一致している必要がある
// (config.hppのCUBEMARS_MIT_*参照)。
uint16_t floatToUint(float x, float x_min, float x_max, uint8_t bits) {
    x = std::min(std::max(x, x_min), x_max);
    float span = x_max - x_min;
    uint32_t raw = (uint32_t)((x - x_min) * (float)(1UL << bits) / span);
    uint32_t max_raw = (1UL << bits) - 1UL;
    return (uint16_t)std::min(raw, max_raw);
}

// Force Control(MIT)モードの8byte指令フレームを組み立てて送信する
// (マニュアル4.2節、KP(12bit)+KD(12bit)+Position(16bit)+Speed(12bit)+Torque(12bit)の順)。
void sendMitCommand(uint8_t motor_can_id, float pos_rad, float vel_radps, float kp, float kd, float torque_nm) {
    uint16_t p_u = floatToUint(pos_rad, CUBEMARS_MIT_P_MIN_RAD, CUBEMARS_MIT_P_MAX_RAD, 16);
    uint16_t v_u = floatToUint(vel_radps, CUBEMARS_MIT_V_MIN_RADPS, CUBEMARS_MIT_V_MAX_RADPS, 12);
    uint16_t kp_u = floatToUint(kp, CUBEMARS_MIT_KP_MIN, CUBEMARS_MIT_KP_MAX, 12);
    uint16_t kd_u = floatToUint(kd, CUBEMARS_MIT_KD_MIN, CUBEMARS_MIT_KD_MAX, 12);
    uint16_t t_u = floatToUint(torque_nm, CUBEMARS_MIT_T_MIN_NM, CUBEMARS_MIT_T_MAX_NM, 12);

    uint8_t buffer[8];
    buffer[0] = (uint8_t)(kp_u >> 4);                          // KP high 8 bits
    buffer[1] = (uint8_t)(((kp_u & 0xF) << 4) | (kd_u >> 8));  // KP low 4 bits | KD high 4 bits
    buffer[2] = (uint8_t)(kd_u & 0xFF);                        // KD low 8 bits
    buffer[3] = (uint8_t)(p_u >> 8);                           // Position high 8 bits
    buffer[4] = (uint8_t)(p_u & 0xFF);                         // Position low 8 bits
    buffer[5] = (uint8_t)(v_u >> 4);                           // Speed high 8 bits
    buffer[6] = (uint8_t)(((v_u & 0xF) << 4) | (t_u >> 8));    // Speed low 4 bits | Torque high 4 bits
    buffer[7] = (uint8_t)(t_u & 0xFF);                         // Torque low 8 bits
    sendExtended(CUBEMARS_CMD_MIT, motor_can_id, buffer, 8);
}

// [cubemars_ctrlで追加] PCからの有効フレームがCUBEMARS_CMD_TIMEOUT_MS以上
// 途絶えているか。一度も受信していない場合もtrue(Rx_16Dataは全ゼロのままなので、
// ゼロ速度指令を送るという元の起動直後の挙動と変わらない)。
bool commandTimedOut() {
    uint32_t last_rx_ms = 0;
    if (!serialLastValidRxMs(&last_rx_ms)) {
        return true;
    }
    return (millis() - last_rx_ms) > CUBEMARS_CMD_TIMEOUT_MS;
}

// -------- CAN送信 (指令 -> AKシリーズ) -------- //

void sendCommands() {
    // [cubemars_ctrlで追加] シリアル途絶時のフェイルセーフ。最後の指令を保持せず、
    // 全モータへゼロ速度指令(その場停止、ros2canのE-STOPと同じ)を送る。
    // 通信が復帰すれば次の周期から通常の指令に戻る。
    if (commandTimedOut()) {
        for (int m = 0; m < CUBEMARS_MOTOR_COUNT; m++) {
            g_origin_cmd_sent[m] = false;
            sendInt32Command(CUBEMARS_CMD_RPM, kMotorCanId[m], 0);
        }
        return;
    }

    for (int m = 0; m < CUBEMARS_MOTOR_COUNT; m++) {
        int16_t target = Rx_16Data[m];
        int16_t mode = Rx_16Data[4 + m];

        if (mode == CUBEMARS_MODE_SET_ORIGIN) {
            // フラッシュ書き込みを伴うため、ホスト側がmode=3を複数周期保持していても
            // 実際のCANコマンドはエッジ検出で1回だけ送る(毎5ms送り続けると不要な
            // 再送・フラッシュ摩耗になるため)。原点設定完了までは速度/位置/MIT
            // 指令を送らずその場停止のままにする。
            if (!g_origin_cmd_sent[m]) {
                int16_t origin_mode_raw = std::min<int16_t>(std::max<int16_t>(target, 0), 2);
                sendSetOriginCommand(kMotorCanId[m], (uint8_t)origin_mode_raw);
                g_origin_cmd_sent[m] = true;
            }
            continue;
        }
        g_origin_cmd_sent[m] = false;

        if (mode == CUBEMARS_MODE_MIT) {
            float pos_rad = (float)(target * MIT_POSITION_LSB_DEG) * DEG_TO_RAD;
            float vel_radps = (float)(Rx_16Data[MIT_SLOT_VELOCITY + m] * MIT_VELOCITY_LSB_RADPS);
            float kp = (float)(Rx_16Data[MIT_SLOT_KP + m] * MIT_KP_LSB);
            float kd = (float)(Rx_16Data[MIT_SLOT_KD + m] * MIT_KD_LSB);
            float torque_ff = (float)(Rx_16Data[MIT_SLOT_TORQUE_FF + m] * MIT_TORQUE_LSB_NM);
            sendMitCommand(kMotorCanId[m], pos_rad, vel_radps, kp, kd, torque_ff);
        } else if (mode == CUBEMARS_MODE_POSITION) {
            // 0.1deg/LSB(本機スロット) -> 0.0001deg/LSB(CAN指令、マニュアル4.1.5)
            int32_t pos_cmd = (int32_t)target * 1000;
            sendInt32Command(CUBEMARS_CMD_POS, kMotorCanId[m], pos_cmd);
        } else {
            // 10ERPM/LSB(本機スロット) -> 1ERPM/LSB(CAN指令、マニュアル4.1.4)
            int32_t rpm_cmd = (int32_t)target * 10;
            sendInt32Command(CUBEMARS_CMD_RPM, kMotorCanId[m], rpm_cmd);
        }
    }
}

// -------- CAN受信 (AKシリーズ -> 帰還) -------- //

int motorIndexForCanId(uint8_t can_id) {
    for (int m = 0; m < CUBEMARS_MOTOR_COUNT; m++) {
        if (kMotorCanId[m] == can_id)
            return m;
    }
    return -1;
}

void receiveFeedback() {
    twai_message_t rx_msg;

    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        if (!rx_msg.extd || rx_msg.data_length_code != 8)
            continue;

        uint32_t function_id = rx_msg.identifier >> 8;
        uint8_t driver_id = (uint8_t)(rx_msg.identifier & 0xFF);
        if (function_id != CUBEMARS_FEEDBACK_FUNCTION_ID)
            continue;

        int m = motorIndexForCanId(driver_id);
        if (m < 0)
            continue;

        int16_t position = (int16_t)((rx_msg.data[0] << 8) | rx_msg.data[1]);
        int16_t speed = (int16_t)((rx_msg.data[2] << 8) | rx_msg.data[3]);
        int16_t current = (int16_t)((rx_msg.data[4] << 8) | rx_msg.data[5]);
        int8_t temperature = (int8_t)rx_msg.data[6];
        uint8_t error_code = rx_msg.data[7];

        Tx_16Data[m] = position;
        Tx_16Data[4 + m] = speed;
        Tx_16Data[8 + m] = current;
        Tx_16Data[12 + m] = temperature;
        Tx_16Data[16 + m] = error_code;
        // [cubemars_ctrlで追加] 帰還フレーム受信カウンタ(元は未使用スロット20-23)。
        // 値自体に意味は無く「増え続けているか」だけを見る。位置等の値はモータが
        // 止まっていると変化しないため、それだけでは「モータが応答しているか」を
        // ROS側から判定できない(CAN断線・ID違いでも最後の値が残り続ける)。
        // int16で折り返すが、ROS側は差分(変化の有無)しか見ないので問題ない。
        Tx_16Data[20 + m] = (int16_t)(Tx_16Data[20 + m] + 1);
    }
}

} // namespace

void cubemarsInit() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); // AKシリーズは1Mbps固定(マニュアル1.1節)
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[ERR] cubemars: TWAI install failed");
        while (1)
            ;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[ERR] cubemars: TWAI start failed");
        while (1)
            ;
    }
}

void cubemarsTask(void *pvParameters) {
    while (1) {
        recoverBusIfNeeded();
        receiveFeedback();
        sendCommands();
        // 200Hz。ロボマス/センサノードと1Mbpsバスを共有する構成での帯域見積りは
        // README.md参照(1kHzのままだと合計がバス容量を超える)。
        vTaskDelay(5);
    }
}
