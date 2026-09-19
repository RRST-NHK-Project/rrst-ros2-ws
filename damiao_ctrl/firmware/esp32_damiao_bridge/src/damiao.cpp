/*====================================================================
<damiao.cpp>
・Damiao DM2325(モータ) + DM3520(ESC) 用CANドライバの実装ファイル

達妙(Damiao)公式Python実装 (github.com/cmjang/DM_Control_Python,
DM_CAN.py) のCAN ID体系・バイトパッキングを一次情報として実装した、
VEL(速度)モード・POS_VEL(位置+速度)モードのドライバ。MITモードは
Kp/Kd調整が必要で初回動作確認に不向きなため未実装(将来必要になれば
cubemars.cppのsendMitCommand()と同じ考え方で追加できる)。

CANプロトコル (標準11bit ID, 1Mbps):
  Enable/Disable/原点設定 (ID = モータのCAN ID, DLC=8):
    data = [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, cmd]
    cmd: 0xFC=Enable, 0xFD=Disable, 0xFE=原点設定(SetZero)
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

// Damiao制御コマンド (公式DM_CAN.pyのenable()/disable()/set_zero_position()と同一)
constexpr uint8_t CMD_ENABLE = 0xFC;
constexpr uint8_t CMD_DISABLE = 0xFD;

const uint16_t kMotorCanId[DAMIAO_MOTOR_COUNT] = {
    DAMIAO_MOTOR_ID_1,
};

bool g_enabled[DAMIAO_MOTOR_COUNT] = {false};

// CAN未接続/ESC無応答が続くとBus-Offへ遷移するが、TWAIドライバは自動復帰
// しない。ros2canのcan_task.cpp/cubemars.cppと同じ対策。
void recoverBusIfNeeded() {
    constexpr uint32_t CHECK_PERIOD_MS = 100;
    static uint32_t last_check_ms = 0;
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
    } else if (status.state == TWAI_STATE_STOPPED) {
        twai_start();
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
            }
            continue;
        }

        if (!g_enabled[m]) {
            sendControlCmd(motor_can_id, CMD_ENABLE);
            g_enabled[m] = true;
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
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
#if CAN_BITRATE_1MBPS
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
#else
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
#endif
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[ERR] damiao: TWAI install failed");
        while (1)
            ;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[ERR] damiao: TWAI start failed");
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
