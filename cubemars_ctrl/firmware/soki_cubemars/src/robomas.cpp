/*====================================================================
<robomas.cpp>
・MODE_ROBOMAS(DJI RoboMasterシリーズ CANドライバ)の実装ファイル

config.hppのROBOMAS_MOTOR_TYPEで選択した機種(M3508/M2006/GM6020)を、
同一バス上に最大NUM_MOTOR(4)台まで制御する。他機種との混在は非対応。

速度ループ(既存)に加え、CubeMarsのMIT(Force Control)モードと対称的な位置PD制御
モード(MIT)にも対応する。ただしロボマス側ESC(C610/C620)やGM6020はCANで生の電流
指令しか受け付けずアクチュエータ内蔵の位置/トルク制御が無いため、CubeMarsのように
専用CANフレームを別途持つのではなく、位置PD制御ループ自体をこのマイコン側で計算し
既存のsendCurrentCommand()(電流指令)へ渡すだけになる。位置フィードバックは
ロボマス内蔵ロータエンコーダ(angle[]/vel[])を使う(config.hppのMIT関連コメント参照)。
モータごとにcontrol_modeスロットで速度⇔MITを毎周期選択できる。

スロット割り当て (frame_data.hppのTx_16Data/Rx_16Dataを使用、独立デバイスとして
24スロットをそのまま使う。ノード/スロット分配は行わない):

  Rx_16Data (PC -> 本機, 指令):
    0-3:   target        速度モード: target_rpm、出力軸rpm、生値スケール無し(既存/後方互換)
                          MITモード:  目標位置、1deg/LSB(出力軸角度、範囲±32767deg=約±91回転。
                                      0.1deg/LSBだと±9.1回転までしか指令できずz/r軸等では
                                      不足するため粗くしてある。config.hppのROBOMAS_MIT_POSITION_LSB_DEG参照)
    4-7:   control_mode  モータ1-4: 0=速度ループ(既定), 1=MIT(位置PD制御)
                          全ゼロ(E-STOP/未接続時の既定)で0=速度・target=0となり、
                          安全にゼロ速度指令(その場停止)になる(既存動作から変更無し)。
    8-11:  mit_velocity_ff (モータ1-4): MITモード時のみ参照。目標速度FF、1rpm/LSB
    12-15: mit_kp          (モータ1-4): MITモード時のみ参照。比例ゲイン、0.001(A/deg)/LSB
    16-19: mit_kd          (モータ1-4): MITモード時のみ参照。微分ゲイン、0.0001(A/rpm)/LSB
    20-23: mit_current_ff  (モータ1-4): MITモード時のみ参照。電流FF、0.001A/LSB

  Tx_16Data (本機 -> PC, 帰還。速度/MIT共通、変更無し):
    0-3: angle  [0.1deg単位] (出力軸換算、M3508/M2006はギア比込み)
    4-7: velocity [rpm]      (出力軸換算)
    8-11: current [mA単位]   (実電流換算値)
    12-23: 未使用

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "robomas.hpp"
#include "config.hpp"
#include "defs.hpp"
#include "frame_data.hpp"
#include "PID.hpp"
#include <Arduino.h>

namespace {

// PC側(Rx_16Data)のcontrol_mode enum値 (cubemars.cppのCUBEMARS_MODE_*と同じパターン)
constexpr int16_t ROBOMAS_MODE_VELOCITY = 0;
constexpr int16_t ROBOMAS_MODE_MIT = 1;

// MITモード用の追加スロット (target/control_modeは0-7を流用、robomas.cpp先頭コメント参照)
constexpr int MIT_SLOT_VELOCITY_FF = 8;  // 8-11
constexpr int MIT_SLOT_KP = 12;          // 12-15
constexpr int MIT_SLOT_KD = 16;          // 16-19
constexpr int MIT_SLOT_CURRENT_FF = 20;  // 20-23

// -------- 状態量 (CAN受信フィードバック) -------- //
int16_t encoder_count[NUM_MOTOR] = {0};
int16_t rpm[NUM_MOTOR] = {0};
int16_t current_raw[NUM_MOTOR] = {0};
int last_encoder[NUM_MOTOR] = {0};
int rotation_count[NUM_MOTOR] = {0};
long total_encoder[NUM_MOTOR] = {0};
float angle[NUM_MOTOR] = {0};   // 出力軸角度[deg]
float vel[NUM_MOTOR] = {0};     // 出力軸速度[rpm]
float current_a[NUM_MOTOR] = {0}; // 実電流[A]

// -------- 指令値 -------- //
float target_rpm[NUM_MOTOR] = {0};

// -------- 速度PID状態 -------- //
float motor_output_current[NUM_MOTOR] = {0};

unsigned long g_last_pid_time = 0;

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

PIDController vel_pid[NUM_MOTOR] = {
    PIDController(ROBOMAS_KP_VEL, ROBOMAS_KI_VEL, ROBOMAS_KD_VEL, ROBOMAS_MAX_CURRENT_A / ROBOMAS_OUTPUT_GAIN),
    PIDController(ROBOMAS_KP_VEL, ROBOMAS_KI_VEL, ROBOMAS_KD_VEL, ROBOMAS_MAX_CURRENT_A / ROBOMAS_OUTPUT_GAIN),
    PIDController(ROBOMAS_KP_VEL, ROBOMAS_KI_VEL, ROBOMAS_KD_VEL, ROBOMAS_MAX_CURRENT_A / ROBOMAS_OUTPUT_GAIN),
    PIDController(ROBOMAS_KP_VEL, ROBOMAS_KI_VEL, ROBOMAS_KD_VEL, ROBOMAS_MAX_CURRENT_A / ROBOMAS_OUTPUT_GAIN),
};

float constrainFloat(float val, float min_val, float max_val) {
    if (val < min_val)
        return min_val;
    if (val > max_val)
        return max_val;
    return val;
}

// -------- CAN送信 (指令 -> ESC) -------- //

void sendCurC620(const float cur_array[NUM_MOTOR]) {
    // C620(M3508)/C610(M2006)共通、ID1-4宛の一括電流指令フレーム
    twai_message_t tx{};
    tx.identifier = 0x200;
    tx.extd = 0;
    tx.rtr = 0;
    tx.data_length_code = 8;

    for (int i = 0; i < NUM_MOTOR; i++) {
#if ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_M3508
        constexpr float MAX_CUR = 20.0f;
        constexpr int16_t MAX_CUR_VAL = 16384;
#else // ROBOMAS_MOTOR_M2006
        constexpr float MAX_CUR = 10.0f;
        constexpr int16_t MAX_CUR_VAL = 10000;
#endif
        float amp = constrainFloat(cur_array[i], -MAX_CUR, MAX_CUR);
        int16_t val = static_cast<int16_t>(amp * (MAX_CUR_VAL / MAX_CUR));

        tx.data[i * 2] = (uint8_t)(val >> 8);
        tx.data[i * 2 + 1] = (uint8_t)(val & 0xFF);
    }

    if (twai_transmit(&tx, pdMS_TO_TICKS(20)) != ESP_OK) {
        logTransmitFailure("[ERR] robomas: twai_transmit(0x200) failed");
    }
}

void sendCurGm6020(const float cur_array[NUM_MOTOR]) {
    twai_message_t tx{};
    tx.identifier = 0x1FE;
    tx.extd = 0;
    tx.rtr = 0;
    tx.data_length_code = 8;

    constexpr float MAX_CUR = 3.0f;
    constexpr int16_t MAX_CUR_VAL = 16384;

    for (int i = 0; i < NUM_MOTOR; i++) {
        float amp = constrainFloat(cur_array[i], -MAX_CUR, MAX_CUR);
        int16_t val = static_cast<int16_t>(amp * (MAX_CUR_VAL / MAX_CUR));

        tx.data[i * 2] = (uint8_t)(val >> 8);
        tx.data[i * 2 + 1] = (uint8_t)(val & 0xFF);
    }

    if (twai_transmit(&tx, pdMS_TO_TICKS(20)) != ESP_OK) {
        logTransmitFailure("[ERR] robomas: twai_transmit(0x1FE) failed");
    }
}

void sendCurrentCommand(const float cur_array[NUM_MOTOR]) {
#if ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_GM6020
    sendCurGm6020(cur_array);
#else
    sendCurC620(cur_array);
#endif
}

// -------- CAN受信 (ESC -> 帰還) -------- //

void receiveFeedback() {
    twai_message_t rx_msg;

    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        if (rx_msg.data_length_code != 8)
            continue;

#if ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_GM6020
        // GM6020: ID1-4 の帰還フレームは 0x205-0x208
        if (rx_msg.identifier < 0x205 || rx_msg.identifier > 0x208)
            continue;
        int m = rx_msg.identifier - 0x205;
#else
        // C620/C610: ID1-4 の帰還フレームは 0x201-0x204
        if (rx_msg.identifier < 0x201 || rx_msg.identifier > 0x204)
            continue;
        int m = rx_msg.identifier - 0x201;
#endif
        if (m < 0 || m >= NUM_MOTOR)
            continue;

        encoder_count[m] = (int16_t)(rx_msg.data[0] << 8 | rx_msg.data[1]);
        rpm[m] = (int16_t)(rx_msg.data[2] << 8 | rx_msg.data[3]);
        current_raw[m] = (int16_t)(rx_msg.data[4] << 8 | rx_msg.data[5]);

        // エンコーダ回転数計算 (周回検出)
        int diff = encoder_count[m] - last_encoder[m];
        if (diff > HALF_ENCODER)
            rotation_count[m]--;
        else if (diff < -HALF_ENCODER)
            rotation_count[m]++;
        last_encoder[m] = encoder_count[m];

        total_encoder[m] = rotation_count[m] * (long)ENCODER_MAX + encoder_count[m];

#if ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_M3508
        angle[m] = total_encoder[m] * (360.0f / (ENCODER_MAX * gear_m3508));
        vel[m] = rpm[m] / gear_m3508;
        current_a[m] = current_raw[m] * 20.0f / 16384.0f;
#elif ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_M2006
        angle[m] = total_encoder[m] * (360.0f / (ENCODER_MAX * gear_m2006));
        vel[m] = rpm[m] / gear_m2006;
        current_a[m] = current_raw[m] * 10.0f / 10000.0f;
#elif ROBOMAS_MOTOR_TYPE == ROBOMAS_MOTOR_GM6020
        angle[m] = total_encoder[m] * (360.0f / ENCODER_MAX); // ダイレクトドライブ、ギア無し
        vel[m] = (float)rpm[m];
        current_a[m] = current_raw[m] * 3.0f / 16384.0f;
#endif
    }
}

} // namespace

void robomasInit() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); // DJI RoboMasterシリーズは1Mbps固定
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[ERR] robomas: TWAI install failed");
        while (1)
            ;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[ERR] robomas: TWAI start failed");
        while (1)
            ;
    }

    g_last_pid_time = micros();

    for (int i = 0; i < NUM_MOTOR; i++) {
        vel_pid[i].reset();
    }
}

void robomasTask(void *pvParameters) {
    while (1) {
        recoverBusIfNeeded();

        // ループ周期は約5ms(200Hz、CubeMars/センサノードと1Mbpsバスを共有するため
        // 指令送信頻度を落としてある)。millis()では量子化誤差がdtに対して無視でき
        // ない比率になるため、引き続きmicros()で測る。
        unsigned long now = micros();
        float dt = (unsigned long)(now - g_last_pid_time) / 1000000.0f;
        if (dt <= 0)
            dt = 0.000001f;
        if (dt > 0.02f)
            dt = 0.02f;
        g_last_pid_time = now;

        receiveFeedback();

        for (int i = 0; i < NUM_MOTOR; i++) {
            int16_t control_mode = Rx_16Data[4 + i];

            if (control_mode == ROBOMAS_MODE_MIT) {
                float target_pos_deg = Rx_16Data[i] * ROBOMAS_MIT_POSITION_LSB_DEG;
                float target_vel_ff_rpm = Rx_16Data[MIT_SLOT_VELOCITY_FF + i] * ROBOMAS_MIT_VELOCITY_FF_LSB_RPM;
                float kp = Rx_16Data[MIT_SLOT_KP + i] * ROBOMAS_MIT_KP_LSB;
                float kd = Rx_16Data[MIT_SLOT_KD + i] * ROBOMAS_MIT_KD_LSB;
                float current_ff = Rx_16Data[MIT_SLOT_CURRENT_FF + i] * ROBOMAS_MIT_CURRENT_FF_LSB_A;

                float pos_error = target_pos_deg - angle[i];
                float vel_error = target_vel_ff_rpm - vel[i];
                motor_output_current[i] = constrainFloat(
                    kp * pos_error + kd * vel_error + current_ff,
                    -ROBOMAS_MAX_CURRENT_A, ROBOMAS_MAX_CURRENT_A);

                // 速度モードへ戻したときにI項が汚染されていないよう、MIT中は
                // 毎周期リセットして待機させておく。
                vel_pid[i].reset();
            } else {
                target_rpm[i] = Rx_16Data[i];
                vel_pid[i].set_target(target_rpm[i]);
                float vel_out = vel_pid[i].update(vel[i], dt);
                motor_output_current[i] = constrainFloat(vel_out * ROBOMAS_OUTPUT_GAIN, -ROBOMAS_MAX_CURRENT_A, ROBOMAS_MAX_CURRENT_A);
            }
        }

        sendCurrentCommand(motor_output_current);

        for (int i = 0; i < NUM_MOTOR; i++) {
            // 0.1deg単位。int16なので±3276.7deg(約±9回転)で飽和する点に注意。
            Tx_16Data[i] = (int16_t)(angle[i] * 10.0f);
            Tx_16Data[4 + i] = (int16_t)vel[i];
            Tx_16Data[8 + i] = (int16_t)(current_a[i] * 1000.0f);
        }

        // 200Hz。CubeMars/センサノードと1Mbpsバスを共有する構成での帯域見積りは
        // README.md参照(1kHzのままだと合計がバス容量を超える)。
        vTaskDelay(5);
    }
}
