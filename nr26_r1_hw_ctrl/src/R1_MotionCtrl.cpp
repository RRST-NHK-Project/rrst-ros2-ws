/*
R1機構制御
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

// まだ未確認なので絶対に許可なしに起動しないこと！！
// 破壊しても自己責任！！

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>
#include <cstdint>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

// 以下マイコンに合わせて設定
#define OUTPUT_DEVICE_ID 2 // 出力マイコン（モーター制御）のID
#define INPUT_DEVICE_ID  3 // 入力マイコン（マイクロスイッチ）のID
#define TX16NUM 24         // 送信データ数
#define RX16NUM 17         // 受信データ数

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

// スティックのデッドゾーン
#define DEADZONE_L 0.3
#define DEADZONE_R 0.3

// =================================================================
// マイクロスイッチの状態（ID=3のESP32から受信、2ノード間で共有）
// atomic: スレッドセーフに読み書きするため
std::atomic<int16_t> g_micro1_sw{0}; // マイクロスイッチ(上): 1=押されている
std::atomic<int16_t> g_micro2_sw{0}; // マイクロスイッチ(下): 1=押されている
std::atomic<int16_t> g_micro3_sw{0}; // マイクロスイッチ(外側): SW3
std::atomic<int16_t> g_micro4_sw{0}; // マイクロスイッチ(内側): SW4
std::atomic<int16_t> g_enc1_val{0};  // エンコーダ1: data[1]から受信

// フォークリフト座標管理 (EncoderCoordinator)
// エンコーダ減少 -> 座標増加 / エンコーダ増加 -> 座標減少
std::atomic<int64_t> g_abs_coord{0};        // 現在の絶対座標
std::atomic<int16_t> g_last_enc1_val{0};    // 前回のエンコーダ生値（差分計算用）
std::atomic<bool> g_coord_initialized{false}; // 初期化フラグ

// =================================================================
// SwitchInputノード: ID=3のESP32からマイクロスイッチの状態を受信する
// =================================================================
class SwitchInput : public rclcpp::Node {
public:
    SwitchInput()
        : Node("switch_input_" + std::to_string(INPUT_DEVICE_ID)) {

        sw_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(INPUT_DEVICE_ID),
            10,
            std::bind(&SwitchInput::sw_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "SwitchInput: serial_rx_%d を受信開始", INPUT_DEVICE_ID);
    }

private:
    void sw_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        // SW4 (data[12]) まで使用するため、サイズチェックを13以上に変更
        if (msg->data.size() < 13) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "serial_rx_%d: データが短すぎます (%zu)",
                        INPUT_DEVICE_ID, msg->data.size());
            return;
        }

        // マイクロスイッチの値を更新
        // マイクロスイッチの値を更新 (物理配線に合わせて修正: 9=下, 10=上)
        g_micro1_sw = msg->data[10]; // 上端スイッチ(micro1)
        g_micro2_sw = msg->data[9];  // 下端スイッチ(micro2)
        g_micro3_sw = msg->data[11];
        g_micro4_sw = msg->data[12];
        g_enc1_val  = msg->data[1];

        // =============================================================
        // 座標調査用デバッグ実装
        // =============================================================
        int16_t current_enc1 = msg->data[1];

        // 下端スイッチ(data[9])で座標リセット
        if (msg->data[9] != 0) {
            g_abs_coord.store(0);
            g_last_enc1_val.store(current_enc1);
            g_coord_initialized.store(true);
            RCLCPP_INFO(get_logger(), "[COORD RESET!] 下端ボタン押下により 0 にリセットしました");
        } else if (g_coord_initialized.load()) {
            // 差分計算
            int16_t diff = static_cast<int16_t>(current_enc1 - g_last_enc1_val.load());
            
            if (diff != 0) {
                // 現状の定義: 減少->増加 (coord -= diff)
                // ※もし上昇時に数値が減るようなら、ここを += に切り替える
                g_abs_coord = g_abs_coord.load() - static_cast<int64_t>(diff);
                g_last_enc1_val.store(current_enc1);
                
                // 変化があった時だけ詳細ログを出す
                RCLCPP_INFO(get_logger(), "MOVE: enc=%d | diff=%d | raw=%ld | rot=%.3f",
                            (int)current_enc1, (int)diff, g_abs_coord.load(), 
                            (double)g_abs_coord.load() / 65536.0);
            }
        } else {
            g_last_enc1_val.store(current_enc1);
            g_coord_initialized.store(true);
        }

        // --- 通信デバッグ追加 ---
        static uint64_t packet_count = 0;
        packet_count++;

        // 状態変化時のみ即時表示
        static int16_t l9=0, l10=0, l11=0, l12=0;
        if (msg->data[9] != l9 || msg->data[10] != l10 || msg->data[11] != l11 || msg->data[12] != l12) {
            RCLCPP_INFO(get_logger(), "SW Changed! [下(9):%d, 上(10):%d, 外(11):%d, 内(12):%d]", 
                        msg->data[9], msg->data[10], msg->data[11], msg->data[12]);
            l9 = msg->data[9]; l10 = msg->data[10]; l11 = msg->data[11]; l12 = msg->data[12];
        }

        // 定期ダンプに受信件数を追加
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
            "RX Heartbeat (Total:%lu) | Dump: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
            packet_count,
            msg->data[0], msg->data[1], msg->data[2], msg->data[3], 
            msg->data[4], msg->data[5], msg->data[6], msg->data[7], 
            msg->data[8], msg->data[9], msg->data[10], msg->data[11], 
            msg->data[12], msg->data[13], msg->data[14], msg->data[15]);

        // SW3, SW4専用の明示的なデバッグ
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "【通信確認】SW3(外側):%d, SW4(内側):%d", msg->data[11], msg->data[12]);
    }

    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sw_sub_;
};

// =================================================================
// HardWareControlノード: ID=2のESP32へモーター指令を送信する
// =================================================================
class HardWareControl : public rclcpp::Node {
public:
    HardWareControl()
        : Node("hardware_control_" + std::to_string(OUTPUT_DEVICE_ID)) {

        // 配列を0で初期化
        data_.assign(TX16NUM, 0);
        /*
        マイコンに送信される配列"data_"
        debug: 機能未割り当て, MD: モータードライバー, TR: トランジスタ
        | data[n] | 詳細 | 範囲 |
        | ---- | ---- | ---- |
        | data[0] | debug | 0 or 1 |
        | data[1] | MD1 | -100 ~ 100 |
        | data[2] | MD2 | -100 ~ 100 |
        | data[3] | MD3 | -100 ~ 100 |
        | data[4] | MD4 | -100 ~ 100 |
        | data[5] | MD5 | -100 ~ 100 |
        | data[6] | MD6 | -100 ~ 100 |
        | data[7] | MD7 | -100 ~ 100 |
        | data[8] | MD8 | -100 ~ 100 |
        | data[9] | Servo1 | 0 ~ 270 |
        | data[10] | Servo2 | 0 ~ 270 |
        | data[11] | Servo3 | 0 ~ 270 |
        | data[12] | Servo4 | 0 ~ 270 |
        | data[13] | Servo5 | 0 ~ 270 |
        | data[14] | Servo6 | 0 ~ 270 |
        | data[15] | Servo7 | 0 ~ 270 |
        | data[16] | Servo8 | 0 ~ 270 |
        | data[17] | TR1 | 0 or 1|
        | data[18] | TR2 | 0 or 1|
        | data[19] | TR3 | 0 or 1|
        | data[20] | TR4 | 0 or 1|
        | data[21] | TR5 | 0 or 1|
        | data[22] | TR6 | 0 or 1|
        | data[23] | TR7 | 0 or 1|
        | data[24] | TR8 | 0 or 1|
        */

        // joyノードのSubscribe
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&HardWareControl::ps4_listener_callback, this, std::placeholders::_1));

        // seial_bridgeへpublish
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(OUTPUT_DEVICE_ID), 10);

        // timer_callbackを呼び出すタイマーを作成
        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&HardWareControl::publisher_timer_callback, this));

        // sensor_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        //     "serial_rx_" + std::to_string(device_id_),
        //     10,
        //     std::bind(&HardWareControl::sensor_callback,
        //               this,
        //               std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "HardWareControl: serial_tx_%d 送信開始", OUTPUT_DEVICE_ID);
    }

private:
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        // float LS_X = -1 * msg->axes[0];
        // float LS_Y = msg->axes[1];
        // float RS_X = -1 * msg->axes[3];
        // float RS_Y = msg->axes[4];

        bool CROSS = msg->buttons[0];
        bool CIRCLE = msg->buttons[1];
        bool TRIANGLE = msg->buttons[2];
        bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        bool L1 = msg->buttons[4];
        bool R1 = msg->buttons[5];

        // float L2_DIGITAL = (-1 * msg->axes[2] + 1) / 2;
        // float R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2;

        // bool L2 = msg->buttons[6];
        // bool R2 = msg->buttons[7];

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        // static bool last_option = false;
        // static bool option_latch = false;

        // static bool last_share = false;
        // static bool share_latch = false;

        // マイクロスイッチの状態をグローバル変数から取得
        int16_t micro1_sw = g_micro1_sw.load(); 
        int16_t micro2_sw = g_micro2_sw.load(); 
        int16_t micro3_sw = g_micro3_sw.load(); 
        int16_t micro4_sw = g_micro4_sw.load(); 

        // 制御ノード側のデバッグログ
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "【制御ノード表示】SW状態: 上=%d (%s), 下=%d (%s), 外=%d (%s), 内=%d (%s)", 
            micro1_sw, micro1_sw ? "停止" : "通常",
            micro2_sw, micro2_sw ? "停止" : "通常",
            micro3_sw, micro3_sw ? "停止" : "通常",
            micro4_sw, micro4_sw ? "停止" : "通常");

        // 以降、配列data_を操作する
        // =================================================================
        // CROSS:「マガジン回転」※未確認！絶対に起動するな！！自己責任！！
        // ボタンを一回押すごとにサーボモーターを45°づつ回転する
        // =================================================================

        static int MAG_SERVO_ANGLE[] = {270, 228, 186, 146, 100, 58, 13, 141, 95, 50, 53, 5, 8};
        static int BAR_PUSH_ANGLE = 10;
        static int BAR_HOLD_ANGLE = 43;
        static int BAR_RELE_ANGLE = 245;
        static int cross_pre = 0;
        static int CROSS_PUSH_COUNT = 0;
        static int CROSS_PUSH_MAX = 20;
        static int ang_1 = 13;
        static int ang_2 = 90;

        if (CROSS == 1 && cross_pre == 0) {
            CROSS_PUSH_COUNT = (CROSS_PUSH_COUNT + 1) % CROSS_PUSH_MAX;
        }
        if (CROSS_PUSH_COUNT == 0) {
            data_[9] = MAG_SERVO_ANGLE[0];
            data_[12] = ang_1;
        }
        if (CROSS_PUSH_COUNT == 1) {
            data_[9] = MAG_SERVO_ANGLE[1];
        }
        if (CROSS_PUSH_COUNT == 2) {
            data_[9] = MAG_SERVO_ANGLE[2];
        }
        if (CROSS_PUSH_COUNT == 3) {
            data_[9] = MAG_SERVO_ANGLE[3];
        }
        if (CROSS_PUSH_COUNT == 4) {
            data_[10] = BAR_HOLD_ANGLE;
        }
        if (CROSS_PUSH_COUNT == 5) {
            data_[10] = BAR_PUSH_ANGLE;
        }
        if (CROSS_PUSH_COUNT == 6) {
            data_[10] = BAR_RELE_ANGLE;
        }
        if (CROSS_PUSH_COUNT == 7) {
            data_[9] = MAG_SERVO_ANGLE[8];
            // data_[12] = ang_2;
        }
        if (CROSS_PUSH_COUNT == 8) {
            // data_[9] = MAG_SERVO_ANGLE[5];
            data_[10] = BAR_HOLD_ANGLE;
            // data_[12] = ang_1;
        }
        if (CROSS_PUSH_COUNT == 9) {
            data_[10] = BAR_PUSH_ANGLE;
        }
         if (CROSS_PUSH_COUNT == 10) {
            data_[10] = BAR_RELE_ANGLE;
        }
        if (CROSS_PUSH_COUNT == 11) {
            data_[9] = MAG_SERVO_ANGLE[9];
            // data_[12] = ang_2;
        }
        if (CROSS_PUSH_COUNT == 12) {
            data_[9] = MAG_SERVO_ANGLE[5];
            data_[10] = BAR_HOLD_ANGLE;
            // data_[12] = ang_1;
        }
        if (CROSS_PUSH_COUNT == 13) {
            data_[10] = BAR_PUSH_ANGLE;
        }
         if (CROSS_PUSH_COUNT == 14) {
            data_[10] = BAR_RELE_ANGLE;
        }
        if (CROSS_PUSH_COUNT == 15) {
            data_[9] = MAG_SERVO_ANGLE[10];
            // data_[12] = ang_2;
        }
        if (CROSS_PUSH_COUNT == 16) {
            data_[9] = MAG_SERVO_ANGLE[6];
            data_[10] = BAR_HOLD_ANGLE;
            // data_[12] = ang_1;
        }
        if (CROSS_PUSH_COUNT == 17) {
            data_[10] = BAR_PUSH_ANGLE;
            data_[9] = MAG_SERVO_ANGLE[5];
        }
         if (CROSS_PUSH_COUNT == 18) {
            data_[10] = BAR_RELE_ANGLE;
        }
        if (CROSS_PUSH_COUNT == 19) {
            data_[9] = MAG_SERVO_ANGLE[6];
            // data_[12] = ang_2;
        }

        cross_pre = CROSS;

        // =================================================================
        // CIRCLE:「ワーク回収用機構」 ※未確認!起動は自己責任！！
        // 
        // ハンドの上下にリミッターついてるか不明だったので一旦スピード０にしてます。
        // あとdata_ の番号は適当に決めたので後で確認お願いします
        // =================================================================

        static int circle_pre = 0;
        static int CIRCLE_PUSH_COUNT = 0;
        static int CIRCLE_PUSH_MAX = 6;

        static int MOVE_SPEED = 50;
        static int VACUUM_SPEED = 50;

        if (CIRCLE == 1 && circle_pre == 0) {
            CIRCLE_PUSH_COUNT = (CIRCLE_PUSH_COUNT + 1) % CIRCLE_PUSH_MAX;
        }
        if (CIRCLE_PUSH_COUNT == 0) {
            data_[1] = 0;
            data_[4] = 0;
            data_[17] = 0;
        }
        if (CIRCLE_PUSH_COUNT == 1) {
            data_[1] = MOVE_SPEED;
            if (micro3_sw == 1) data_[1] = 0; // 外側SWで停止
        }                       
        if (CIRCLE_PUSH_COUNT == 2) {
            data_[4] = VACUUM_SPEED;
        }
        if (CIRCLE_PUSH_COUNT == 3) {
            data_[1] = MOVE_SPEED * -1;
            if (micro4_sw == 1) data_[1] = 0; // 内側SWで停止
        }
        if (CIRCLE_PUSH_COUNT == 4) {
            data_[4] = 0;             
        }
        if (CIRCLE_PUSH_COUNT == 5) {
            data_[17] = 1;
        }
        if (CIRCLE_PUSH_COUNT == 6) {
            data_[17] = 0;
        }   

        circle_pre = CIRCLE;

        // =================================================================
        // 元CIRCLE:「棒ホールド機構」※CROSSと統合済み
        // ボタンを一回押すごとに2つのサーボモーターの角度状態を同時に変化させる
        // =================================================================
        // angle = 10のとき最下部までお仕込み
        // angle = 245のときマガジンに戻してる

        // static int BAR_PUSH_ANGLE = 10;
        // static int BAR_HOLD_ANGLE = 43;
        // static int BAR_RELE_ANGLE = 245;
        // static int circle_pre = 0;
        // static int CIRCLE_PUSH_COUNT = 0;
        // static int CIRCLE_PUSH_MAX = 3;

        // if (CIRCLE == 1 && circle_pre == 0) {
        //     CIRCLE_PUSH_COUNT = (CIRCLE_PUSH_COUNT + 1) % CIRCLE_PUSH_MAX;
        // }
        // if (CIRCLE_PUSH_COUNT == 0) {
        //     data_[10] = BAR_RELE_ANGLE;
        // }
        // if (CIRCLE_PUSH_COUNT == 1) {
        //     data_[10] = BAR_HOLD_ANGLE;
        // }
        // if (CIRCLE_PUSH_COUNT == 2) {
        //     data_[10] = BAR_PUSH_ANGLE;
        // }

        // circle_pre = CIRCLE;

        // =================================================================
        // TRIANGLE:「棒取捨選択」
        // ※とりあえずこの機能も統合
        // =================================================================

        // static int triangle_pre = 0;
        // static int tri_count = 0;
        // static int tri_state;
        // static int ang_1 = 13;
        // static int ang_2 = 90;

        // if (TRIANGLE == 1 && triangle_pre == 0)
        //     tri_count++;

        // tri_state = tri_count % 2;

        // if (tri_state == 0) {
        //     data_[12] = ang_1;
        // } else if (tri_state == 1) {
        //     data_[12] = ang_2;
        // }

        // triangle_pre = TRIANGLE;

        // =================================================================
        // SQUARE:「棒ロック」
        // ボタンを一回押すごとに2つのサーボモーターの角度状態を同時に変化させる
        // （安全上の観点から現時点では統合不可）
        // =================================================================

        static int BAR_BTM_HOLD_ANGLE = 0;
        static int BAR_BTM_ANGLE = 150;
        static int square_pre = 0;
        static int SQUARE_PUSH_COUNT = 0;
        static int SQUARE_PUSH_MAX = 2;

        if (SQUARE == 1 && square_pre == 0) {
            SQUARE_PUSH_COUNT = (SQUARE_PUSH_COUNT + 1) % SQUARE_PUSH_MAX;
        }
        if (SQUARE_PUSH_COUNT == 0) {
            data_[11] = BAR_BTM_ANGLE;
        }
        if (SQUARE_PUSH_COUNT == 1) {
            data_[11] = BAR_BTM_HOLD_ANGLE;
        }

        square_pre = SQUARE;

        // =================================================================
        // UP,DOWN:「槍押上機構」　※実験のため一時コメントアウト中
        // ボタンを押し続けると槍を押し上げるモーターが回転し続ける
        // =================================================================

        // data_[3] = UP;
        // if (UP) {
        //     data_[1] = 50;
        // } else if (DOWN) {
        //     data_[1] = -50;
        // } else {
        //     data_[1] = 0;
        // }

        // =================================================================
        // L1,R1:「フォークリフト上下」
        // 絶対座標に基づく減速 + マイクロスイッチによる方向制限
        //
        // 座標: 下端スイッチ押下で 0 にリセット
        //   座標 <= 3 → 減速 (正回転=30, 逆回転=-30)
        //   座標 > 3  → 通常速度 (正回転=100, 逆回転=-100)
        //   上マイクロスイッチ押下 → 正回転禁止、逆回転のみ許可
        //   下マイクロスイッチ押下 → 逆回転禁止、正回転のみ許可
        // =================================================================

        static const int NORMAL_SPEED = 100;  // 通常速度
        static const int SLOW_SPEED   =  30;  // 減速後速度

        int64_t abs_coord = g_abs_coord.load(); // 現在の絶対座標
        double rot_units = static_cast<double>(abs_coord) / 65536.0; // 回転単位 (1周期=1.0)

        // ヒステリシス（遊び）を持たせた減速ゾーン判定
        // 下端付近 (0〜3回転) または 上限付近 (27〜30回転) で減速
        static bool is_fork_slow = false;
        if (rot_units <= 3.0 || rot_units >= 27.0) {
            is_fork_slow = true;
        } else if (rot_units >= 6.0 && rot_units <= 24.0) {
            is_fork_slow = false;
        }
        bool in_slow_zone = is_fork_slow;

        // 回転方向に応じた速度を決定
        int fwd_speed = in_slow_zone ?  SLOW_SPEED :  NORMAL_SPEED;  // 正回転速度
        int rev_speed = in_slow_zone ? -SLOW_SPEED : -NORMAL_SPEED;  // 逆回転速度

        if (micro2_sw == 1 /* || rot_units >= 30.0 */) {
            // ★上端制限: 正回転(L1)禁止 (デバッグのためソフトリミット一時解除)
            if (R1 == 1) {
                data_[2] = rev_speed;
            } else {
                data_[2] = 0;
            }
        } else if (micro1_sw == 1 /* || rot_units <= 0.0 */) {
            // ★下端制限: 逆回転(R1)禁止 (デバッグのためソフトリミット一時解除)
            if (L1 == 1) {
                data_[2] = fwd_speed;
            } else {
                data_[2] = 0;
            }
        } else if (L1 == 1) {
            // 通常: 正回転(L1)
            data_[2] = fwd_speed;
        } else if (R1 == 1) {
            // 通常: 逆回転(R1)
            data_[2] = rev_speed;
        } else {
            // ボタンなし: 停止
            data_[2] = 0;
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "【フォーク制御】回転数=%.2f, 減速=%s, 上SW=%d, 下SW=%d, data[2]=%d",
            rot_units, in_slow_zone ? "ON" : "OFF",
            micro2_sw, micro1_sw, data_[2]);

        // =================================================================
        // LR
        // マガジン調整用
        // static int MAG_ADJ_STEP = 5;
        // static int left_pre = 0;
        // static int right_pre = 0;
        // static int mag_servo_angle = 270;

        // if (LEFT == 1 && left_pre == 0) {
        //     mag_servo_angle -= MAG_ADJ_STEP;
        //     data_[9] = mag_servo_angle;
        // }
        // left_pre = LEFT;
        // if (RIGHT == 1 && right_pre == 0) {
        //     mag_servo_angle += MAG_ADJ_STEP;
        //     data_[9] = mag_servo_angle;
        // }
        // right_pre = RIGHT;
        // =================================================================

        // デバッグ用
        // RCLCPP_INFO(
        //     get_logger(),
        //     "data_[1-4]=[%d,%d,%d,%d], data_[9-12]=[%d,%d,%d,%d]",
        //     data_[1], data_[2], data_[3], data_[4],
        //     data_[9], data_[10], data_[11], data_[12]);

        // 配列操作ここまで
    }

    // publish
    void publisher_timer_callback() {
        std_msgs::msg::Int16MultiArray msg;

        msg.data = data_;

        publisher_->publish(msg);
    }

    // void
    // sensor_callback(
    //     const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    //     // 最低限：サイズチェック
    //     if (msg->data.size() < RX16NUM) {
    //         RCLCPP_WARN(this->get_logger(),
    //                     "serial_rx_%d: data too short (%zu)",
    //                     device_id_, msg->data.size());
    //         return;
    //     }

        // int16_t ENC1 = msg->data[1];
        // int16_t ENC2 = msg->data[2];
        // int16_t ENC3 = msg->data[3];
        // int16_t ENC4 = msg->data[4];
        // int16_t ENC5 = msg->data[5];
        // int16_t ENC6 = msg->data[6];
        // int16_t ENC7 = msg->data[7];
        // int16_t ENC8 = msg->data[8];

        // int16_t SW1 = msg->data[9];
        // int16_t SW2 = msg->data[10];
        // int16_t SW3 = msg->data[11];
        // int16_t SW4 = msg->data[12];
        // int16_t SW5 = msg->data[13];
        // int16_t SW6 = msg->data[14];
        // int16_t SW7 = msg->data[15];
        // int16_t SW8 = msg->data[16];

        // 以降、受信データを使った処理を記述
        // エンコーダースイッチの状態を保存（モーター制御で使用）
        // micro1_sw_ = SW1;
        // micro2_sw_ = SW2;

        // デバッグ: マイクロスイッチの受信値を確認
        // RCLCPP_INFO(get_logger(),
        //             "[マイクロSW] 上(L1禁止用)=%s  下(R1禁止用)=%s",
        //             micro1_sw_ ? "★押されている" : "　押されていない",
        //             micro2_sw_ ? "★押されている" : "　押されていない");

        // 受信データ処理ここまで
    // }

    uint8_t device_id_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<int16_t> data_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet R1 Motion Ctrl";
    int result = std::system(figletout.c_str());
    if (result != 0) {
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
        std::cerr << "Please install 'figlet' with the following command:"
                  << std::endl;
        std::cerr << "sudo apt install figlet" << std::endl;
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
    }

    rclcpp::executors::MultiThreadedExecutor exec;

    // ID=2: モーター出力ノード
    auto hardware_control = std::make_shared<HardWareControl>();
    exec.add_node(hardware_control);

    // ID=3: マイクロスイッチ入力ノード
    auto switch_input = std::make_shared<SwitchInput>();
    exec.add_node(switch_input);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
