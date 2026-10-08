/*====================================================================
<damiao.cpp>
・Damiao DM2325(モータ) + DM3520(ESC) 用CANドライバの実装ファイル

達妙(Damiao)公式Python実装 (github.com/cmjang/DM_Control_Python,
DM_CAN.py) のCAN ID体系・バイトパッキングを一次情報として実装した、
VEL(速度)モード・POS_VEL(位置+速度)モードのドライバ。MITモードは
Kp/Kd調整が必要で初回動作確認に不向きなため未実装(将来必要になれば
cubemars.cppのsendMitCommand()と同じ考え方で追加できる)。

CANプロトコル (標準11bit ID, 1Mbps)。2026-10-01、ユーザーが過去に実機で
動作確認済みのArduinoリファレンス実装(Arduino Mega + MCP2515 + PS4USB)
を一次情報として全面的に見直した:
  Enable/Disable/ClearError/原点設定 (ID = モータのCAN IDそのまま, DLC=8):
    data = [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, cmd]
    cmd: 0xFC=Enable, 0xFD=Disable, 0xFB=ClearError, 0xFE=原点設定(SetZero)
  モードレジスタ書き込み (ID = 0x7FF固定, DLC=8):
    data = [motor_id下位8bit, motor_id上位3bit, 0x55, RID(=10/CMODE), value(int32 LE)]
    value: 2=POS_VELモード, 3=VELモード(要検証)。
    制御フレームを送る前に、このレジスタ書き込みでモータ側の受付モードを
    切り替えておかないと、モータは制御フレームに一切反応しない
    (今回最大の見落としだった箇所)。
  VELモード (ID = 0x200 + モータCAN ID, DLC=8):
    data[0:4] = 目標速度[rad/s] (float32 little-endian), data[4:8] = 0
  POS_VELモード (ID = 0x100 + モータCAN ID, DLC=8):
    data[0:4] = 目標位置[rad] (float32 LE), data[4:8] = 目標速度[rad/s] (float32 LE)
  フィードバック (ID = モータのCAN ID, DLC=8。MIT/Cheetah系で共通の
                  固定小数点ステータスフレーム。config.hppのDAMIAO_P_MAX/
                  V_MAX/T_MAXでデコードするため、実機確認前は数値がズレる
                  可能性がある点に注意):
    byte0: [エラーコード(上位4bit)][モータID下位4bit]
    byte1-2: 位置 (uint16, 0~65535 -> -P_MAX~P_MAX)
    byte3, byte4上位4bit: 速度 (uint12, 0~4095 -> -V_MAX~V_MAX)
    byte4下位4bit, byte5: トルク (uint12, 0~4095 -> -T_MAX~T_MAX)
    byte6: MOS温度[degC], byte7: ロータ温度[degC]

スロット割り当て (damiao_ctrl/include/damiao_ctrl/slot_layout.hppと一致
させること。6スロット/モータ、モータnのオフセット = (n-1)*6):
  Rx_16Data (PC -> 本機, 指令):
    +0: mode_word (0=DISABLE, 1=VEL, 2=POS_VEL)。全ゼロ=DISABLEが安全な
        既定値(Damiaoモータは明示的なEnableコマンドを送るまでCAN指令を
        無視するため)。
    +1: target (0.001単位/LSB。VELならrad/s、POS_VELならrad)
    +2: vel_limit (0.001 rad/s/LSB。POS_VELモードでのみ使用)
  Tx_16Data (本機 -> PC, 帰還):
    +3: position [0.001 rad/LSB]
    +4: velocity [0.001 rad/s/LSB]
    +5: torque   [0.001 N・m/LSB]
====================================================================*/

#include "damiao.hpp"
#include "config.hpp"
#include "frame_data.hpp"
#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace {

constexpr int SLOTS_PER_MOTOR = 6;
constexpr double SLOT_SCALE = 0.001; // 1LSB = 0.001 (rad, rad/s, N・m 共通)

// PC側(Rx_16Data)のmode_word enum値。damiao_ctrl/slot_layout.hppと一致させること。
constexpr int16_t MODE_DISABLED = 0;
constexpr int16_t MODE_VEL = 1;
constexpr int16_t MODE_POS_VEL = 2;

// Damiao制御コマンド (公式DM_CAN.pyのenable()/disable()/set_zero_position()と同一)。
// 2026-10-01、ユーザーが過去に実機で動作確認済みのArduinoリファレンス
// 実装(Arduino Mega + MCP2515 + PS4USB)で確認: Enable/Disable/ClearError
// の宛先CAN IDはmotor_can_idそのまま(オフセット無し)。以前「公式STM32
// サンプル(DM3519一拖四)」を参考にmotor_can_id+モードオフセットへ変更
// したが、DM3519は別系統の製品であり誤りだった。
constexpr uint8_t CMD_ENABLE = 0xFC;
constexpr uint8_t CMD_DISABLE = 0xFD;
constexpr uint8_t CMD_CLEAR_ERROR = 0xFB;
constexpr uint8_t CMD_SET_ZERO = 0xFE;

// パラメータ書き込み(レジスタ)コマンド。固定CAN ID 0x7FFへ
// {motor_id_lo, motor_id_hi, 0x55, RID, d0,d1,d2,d3} を送る。
// RID_CMODE(=10)に制御モード値を書き込むことで、モータ側の受付モードを
// 切り替える。この書き込みを行わずに制御フレームだけ送っても、モータは
// 反応しない(今回のバグの本命)。
constexpr uint8_t RID_CMODE = 10;
constexpr int32_t CMODE_POS_VEL = 2;
constexpr int32_t CMODE_VEL = 3; // TODO: 要実機確認(MIT=1,POS_VEL=2から類推。要検証)

const uint16_t kMotorCanId[DAMIAO_MOTOR_COUNT] = {
    DAMIAO_MOTOR_ID_1,
};

bool g_enabled[DAMIAO_MOTOR_COUNT] = {false};
// 現在Enable済みのモード(MODE_VEL/MODE_POS_VEL)。モードが変わったら
// 一旦Disable+再初期化(モードレジスタ書き込み)してからEnableし直す。
int16_t g_enabled_mode[DAMIAO_MOTOR_COUNT] = {MODE_DISABLED};

void logTransmitFailure(const char *message);

bool installTwaiDriver() {
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
#if CAN_BITRATE_1MBPS
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
#else
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
#endif
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

// CAN未接続/ESC無応答が続くとBus-Offへ遷移するが、TWAIドライバは自動復帰
// しない。ros2canのcan_task.cpp/cubemars.cppと同じ対策。
//
// さらに、既知のESP-IDF TWAIドライバのバグ(espressif/esp-idf#19068)への
// 対策も兼ねる: TXパイプラインがレースコンディションで永久にスタックする
// ことがあり、この場合エラーカウンタは0のまま・bus-offにもならず、
// twai_transmit()がESP_ERR_TIMEOUTを返し続けるだけになる(=タスク側の
// エラーカウンタ監視だけでは検知できない)。msgs_to_txが一定時間変化
// しない状態が続いたら、詰まっていると判断してドライバを再インストール
// する。
void recoverBusIfNeeded() {
    constexpr uint32_t CHECK_PERIOD_MS = 100;
    constexpr uint32_t STUCK_THRESHOLD_MS = 300; // msgs_to_txが変化しないまま許容する時間
    static uint32_t last_check_ms = 0;
    static uint32_t last_msgs_to_tx_change_ms = 0;
    static uint32_t last_msgs_to_tx = 0;

    const uint32_t now_ms = millis();
    if (now_ms - last_check_ms < CHECK_PERIOD_MS) {
        return;
    }
    last_check_ms = now_ms;

    twai_status_info_t status{};
    if (twai_get_status_info(&status) != ESP_OK) {
        return;
    }
    if (status.state == TWAI_STATE_BUS_OFF) {
        twai_initiate_recovery();
        return;
    }
    if (status.state == TWAI_STATE_STOPPED) {
        twai_start();
        return;
    }

    if (status.msgs_to_tx != last_msgs_to_tx) {
        last_msgs_to_tx = status.msgs_to_tx;
        last_msgs_to_tx_change_ms = now_ms;
        return;
    }
    if (status.msgs_to_tx > 0 && (now_ms - last_msgs_to_tx_change_ms) > STUCK_THRESHOLD_MS) {
        // 送信キューに滞留したまま進まない = TXパイプラインのスタック。
        // ドライバを再インストールして復旧する(滞留していたフレームは失われる)。
        logTransmitFailure("[ERR] damiao: TWAI TX stuck, reinstalling driver");
        twai_stop();
        twai_driver_uninstall();
        installTwaiDriver();
        last_msgs_to_tx = 0;
        last_msgs_to_tx_change_ms = now_ms;
    }
}

// twai_transmit失敗ログはserialTaskのバイナリフレームと同じUARTに出るため、
// 頻発すると混入してホスト側のフレーム同期を壊す。間隔を絞って出す。
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

void sendStandard(uint32_t can_id, const uint8_t *data, uint8_t len) {
    twai_message_t tx{};
    tx.identifier = can_id;
    tx.extd = 0; // Damiaoは標準11bit ID
    tx.rtr = 0;
    tx.data_length_code = len;
    for (uint8_t i = 0; i < len; i++) {
        tx.data[i] = data[i];
    }
    if (twai_transmit(&tx, pdMS_TO_TICKS(20)) != ESP_OK) {
        logTransmitFailure("[ERR] damiao: twai_transmit failed");
    }
}

void sendControlCmd(uint16_t motor_can_id, uint8_t cmd) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
    sendStandard(motor_can_id, data, 8);
}

// レジスタ書き込みコマンド(固定ID 0x7FF)。
void writeRegister(uint16_t motor_can_id, uint8_t rid, int32_t value) {
    uint8_t can_id_l = motor_can_id & 0xFF;
    uint8_t can_id_h = (motor_can_id >> 8) & 0x07;
    uint8_t data[8] = {can_id_l, can_id_h, 0x55, rid, 0, 0, 0, 0};
    std::memcpy(&data[4], &value, 4);
    sendStandard(0x7FF, data, 8);
}

void floatToBytesLE(float value, uint8_t *out4) {
    std::memcpy(out4, &value, 4);
}

void sendVel(uint16_t motor_can_id, float vel_radps) {
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    floatToBytesLE(vel_radps, data);
    sendStandard(0x200 + motor_can_id, data, 8);
}

void sendPosVel(uint16_t motor_can_id, float pos_rad, float vel_radps) {
    uint8_t data[8];
    floatToBytesLE(pos_rad, data);
    floatToBytesLE(vel_radps, data + 4);
    sendStandard(0x100 + motor_can_id, data, 8);
}

// モード切替+Enableの初期化シーケンス。実機で動作確認済みのArduino
// リファレンス実装(initMotor())と同じ順序: Disable -> ClearError ->
// モードレジスタ書き込み -> Enable。各コマンド間に短い待ち時間を入れる
// (リファレンスは200-500ms待っていたが、ここでは起動時の一度きりの
// 処理なので50msに短縮)。
void initAndEnable(uint16_t motor_can_id, int32_t cmode_value) {
    sendControlCmd(motor_can_id, CMD_DISABLE);
    delay(50);
    sendControlCmd(motor_can_id, CMD_CLEAR_ERROR);
    delay(50);
    writeRegister(motor_can_id, RID_CMODE, cmode_value);
    delay(50);
    sendControlCmd(motor_can_id, CMD_ENABLE);
    delay(50);
}

// -------- CAN送信 (指令 -> DM3520) -------- //

void sendCommands() {
    for (int m = 0; m < DAMIAO_MOTOR_COUNT; m++) {
        const int off = m * SLOTS_PER_MOTOR;
        const int16_t mode = Rx_16Data[off + 0];
        const float target = (float)(Rx_16Data[off + 1] * SLOT_SCALE);
        const float vel_limit = (float)(Rx_16Data[off + 2] * SLOT_SCALE);
        const uint16_t motor_can_id = kMotorCanId[m];

        if (mode != MODE_VEL && mode != MODE_POS_VEL) {
            // 未知の値も含め、DISABLE以外の全てを安全側(無効化)として扱う。
            if (g_enabled[m]) {
                sendControlCmd(motor_can_id, CMD_DISABLE);
                g_enabled[m] = false;
                g_enabled_mode[m] = MODE_DISABLED;
            }
            continue;
        }

        if (!g_enabled[m] || g_enabled_mode[m] != mode) {
            // 初回Enable、またはモードが切り替わった場合は、
            // Disable -> ClearError -> モードレジスタ書き込み -> Enable の
            // 初期化シーケンスをやり直す(実機確認済みのArduinoリファレンス
            // 実装と同じ手順)。これを行わないとモータが制御フレームに
            // 一切反応しない。
            const int32_t cmode_value = (mode == MODE_POS_VEL) ? CMODE_POS_VEL : CMODE_VEL;
            initAndEnable(motor_can_id, cmode_value);
            g_enabled[m] = true;
            g_enabled_mode[m] = mode;
        }

        if (mode == MODE_VEL) {
            sendVel(motor_can_id, target);
        } else { // MODE_POS_VEL
            sendPosVel(motor_can_id, target, vel_limit);
        }
    }
}

// -------- CAN受信 (DM3520 -> 帰還) -------- //

int motorIndexForCanId(uint16_t can_id) {
    for (int m = 0; m < DAMIAO_MOTOR_COUNT; m++) {
        if (kMotorCanId[m] == can_id)
            return m;
    }
    return -1;
}

// DM_CAN.py uint_to_float() と同じ変換式 (x_min~x_maxの範囲へ線形展開)。
float uintToFloat(uint16_t x, float x_min, float x_max, uint8_t bits) {
    const float span = x_max - x_min;
    const float norm = (float)x / (float)((1UL << bits) - 1UL);
    return norm * span + x_min;
}

// int16スロットへの変換。Arduino.hのmin/maxマクロがstd::clampと衝突する
// ため(cubemars.cppと同じ理由)、std::min/std::maxの組み合わせで手書きする。
int16_t physicalToSlot(double physical_value) {
    const long raw = lround(physical_value / SLOT_SCALE);
    const long clamped = std::min<long>(std::max<long>(raw, INT16_MIN), INT16_MAX);
    return (int16_t)clamped;
}

void receiveFeedback() {
    twai_message_t rx_msg;
    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        if (rx_msg.extd || rx_msg.data_length_code != 8) {
            continue; // Damiaoの標準フィードバックフレームではない
        }

        const int m = motorIndexForCanId((uint16_t)rx_msg.identifier);
        if (m < 0) {
            continue;
        }

        const uint8_t *d = rx_msg.data;
        const uint16_t q_uint = (uint16_t)((d[1] << 8) | d[2]);
        const uint16_t dq_uint = (uint16_t)((d[3] << 4) | (d[4] >> 4));
        const uint16_t tau_uint = (uint16_t)(((d[4] & 0x0F) << 8) | d[5]);

        const float position = uintToFloat(q_uint, -DAMIAO_P_MAX, DAMIAO_P_MAX, 16);
        const float velocity = uintToFloat(dq_uint, -DAMIAO_V_MAX, DAMIAO_V_MAX, 12);
        const float torque = uintToFloat(tau_uint, -DAMIAO_T_MAX, DAMIAO_T_MAX, 12);

        const int off = m * SLOTS_PER_MOTOR;
        Tx_16Data[off + 3] = physicalToSlot(position);
        Tx_16Data[off + 4] = physicalToSlot(velocity);
        Tx_16Data[off + 5] = physicalToSlot(torque);
    }
}

} // namespace

void damiaoInit() {
    if (!installTwaiDriver()) {
        Serial.println("[ERR] damiao: TWAI install failed");
        while (1)
            ;
    }
}

void damiaoTask(void *pvParameters) {
    while (1) {
        recoverBusIfNeeded();
        receiveFeedback();
        sendCommands();
        vTaskDelay(5); // 200Hz (ros2canのcubemars.cpp/robomas.cppと同じ周期)
    }
}
