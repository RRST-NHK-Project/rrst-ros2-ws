/*
R2機構制御
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>
#include <atomic>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

// 以下マイコンに合わせて設定
#define TARGET_DEVICE_ID 7 // 宛先マイコンのID
#define TX16NUM 24         // 送信データ数
#define RX16NUM 17         // 受信データ数

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

// スティックのデッドゾーン
#define DEADZONE_L 0.3
#define DEADZONE_R 0.3

std::atomic<int16_t> g_micro1_sw{0}; // マイクロスイッチ(上): 1=押されている
std::atomic<int16_t> g_micro2_sw{0}; // マイクロスイッチ(下): 1=押されている

// フォークリフト座標管理 (EncoderCoordinator)
// エンコーダ減少 -> 座標増加 / エンコーダ増加 -> 座標減少
std::atomic<int32_t> g_rotation_count{0};     // エンコーダの回転数(巻回り数)
std::atomic<int64_t> g_zero_offset{0};        // 下端リセット時の絶対エンコーダ値
std::atomic<int16_t> g_last_enc1_val{0};      // 前回のエンコーダ生値
std::atomic<bool> g_coord_initialized{false}; // 初期化フラグ
std::atomic<int64_t> g_abs_coord{0};          // 最終的な座標(下端=0方向=プラス)


class HardWareControl : public rclcpp::Node {
public:
    HardWareControl(uint8_t device_id)
        : Node("hardware_control_" + std::to_string(device_id)),
          device_id_(device_id) {

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
        */

        // joyノードのSubscribe
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&HardWareControl::ps4_listener_callback, this, std::placeholders::_1));

        // seial_bridgeへpublish
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(device_id_), 10);

        // timer_callbackを呼び出すタイマーを作成
        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&HardWareControl::publisher_timer_callback, this));

        sensor_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(device_id_),
            10,
            std::bind(&HardWareControl::sensor_callback,
                      this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "serial_tx_%d started.", device_id_);
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

        // 以降、配列data_を操作する

        // =================================================================
        // L1,R1:「フォークリフト上下」
        // 絶対座標に基づく減速 + マイクロスイッチによる方向制限
        // =================================================================

        int16_t micro1_sw = g_micro1_sw.load(); 
        int16_t micro2_sw = g_micro2_sw.load(); 

        static const int NORMAL_SPEED = 50;
        static const int SLOW_SPEED   = 30;

        static const double COUNTS_PER_ROTATION = 8000.0;

        int64_t abs_coord = g_abs_coord.load(); 
        double rot_units = static_cast<double>(abs_coord) / COUNTS_PER_ROTATION;

        // ヒステリシス（遊び）を持たせた減速ゾーン判定
        static bool is_fork_slow = false;
        if (rot_units <= 1.0 || rot_units >= 6.0) {
            is_fork_slow = true;
        } else if (rot_units >= 1.5 && rot_units <= 5.5) {
            is_fork_slow = false;
        }
        bool in_slow_zone = is_fork_slow;

        int fwd_speed = in_slow_zone ?  SLOW_SPEED :  NORMAL_SPEED;
        int rev_speed = in_slow_zone ? -SLOW_SPEED : -NORMAL_SPEED;

        if (micro2_sw == 1) {
            // 上端制限(micro2_sw): 正回転(L1)禁止
            if (R1 == 1) {
                data_[2] = rev_speed;
            } else {
                data_[2] = 0;
            }
        } else if (micro1_sw == 1 || rot_units >=7.0) {
            // 下端制限(micro1_sw): 逆回転(R1)禁止
            if (L1 == 1) {
                data_[2] = fwd_speed;
            } else {
                data_[2] = 0;
            }
        } else {
            // 通常
            if (L1 == 1) {
                data_[2] = fwd_speed;
            } else if (R1 == 1) {
                data_[2] = rev_speed;
            } else {
                data_[2] = 0;
            }
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "【フォーク制御】回転数=%.2f, 減速=%s, 上端SW=%d, 下端SW=%d, 出力=%d",
            rot_units, in_slow_zone ? "ON" : "OFF",
            micro2_sw, micro1_sw, data_[2]);
    
        // デバッグ用
        RCLCPP_INFO(
            get_logger(),
            "data_[1-4]=[%d,%d,%d,%d], data_[9-12]=[%d,%d,%d,%d]",
            data_[1], data_[2], data_[3], data_[4],
            data_[9], data_[10], data_[11], data_[12]);

        // 配列操作ここまで
    }

    // publish
    void publisher_timer_callback() {
        std_msgs::msg::Int16MultiArray msg;

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

        int16_t micro1_sw = g_micro1_sw.load(); 
        int16_t micro2_sw = g_micro2_sw.load(); 

        // 上昇中（data_[2] が正の値）かつ 上端スイッチが押されている場合
        if (micro2_sw == 1 && data_[2] > 0) {
            data_[2] = 0;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "【安全装置】上端リミット到達！モーターの上昇を即時遮断しました！");
        }
        
        // 下降中（data_[2] が負の値）かつ 下端スイッチが押されている場合
        if (micro1_sw == 1 && data_[2] < 0) {
            data_[2] = 0;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "【安全装置】下端リミット到達！モーターの下降を即時遮断しました！");
        }

        msg.data = data_;

        publisher_->publish(msg);
    }

    void
    sensor_callback(
        const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        // 最低限：サイズチェック
        if (msg->data.size() < RX16NUM) {
            RCLCPP_WARN(this->get_logger(),
                        "serial_rx_%d: data too short (%zu)",
                        device_id_, msg->data.size());
            return;
        }

        // マイクロスイッチの値を更新 (物理配線に合わせて修正: 9=下, 10=上)
        g_micro1_sw = msg->data[10]; // 上端スイッチ(micro1)
        g_micro2_sw = msg->data[9];  // 下端スイッチ(micro2)

        // =============================================================
        // 座標調査・ラップアラウンド計算実装（過去コード移植版）
        // =============================================================
        int16_t current_enc1 = msg->data[1];

        if (!g_coord_initialized.load()) {
            g_last_enc1_val.store(current_enc1);
            g_coord_initialized.store(true);
        }

        const int HALF_ENCODER = 16384;
        const int64_t ENCODER_MAX = 32768;

        int diff = (int)current_enc1 - (int)g_last_enc1_val.load();
        int32_t r_count = g_rotation_count.load();

        if (diff > HALF_ENCODER) {
            r_count--;
        } else if (diff < -HALF_ENCODER) {
            r_count++;
        }

        g_rotation_count.store(r_count);
        g_last_enc1_val.store(current_enc1);

        int64_t total_encoder = (int64_t)r_count * ENCODER_MAX + (int64_t)current_enc1;

        if (msg->data[9] != 0) {
            g_zero_offset.store(total_encoder);
            RCLCPP_INFO(get_logger(), "[COORD RESET!] 下端ボタン押下により座標0へオフセット設定");
        }

        int64_t zero_offset = g_zero_offset.load();
        int64_t abs_coord = -(total_encoder - zero_offset);
        g_abs_coord.store(abs_coord);

        double rot = (double)abs_coord / 8000.0; 

        if (diff != 0) {
            RCLCPP_INFO(get_logger(), 
                "\n--- ROTATION DEBUG ---\n"
                "  生値の変化 : %d -> %d (diff: %d)\n"
                "  デジタルラップ : %d 回\n"
                "  絶対カウント   : %ld\n"
                "  現在回転数     : %.3f 回転 (1周8000)\n"
                "----------------------", 
                (int)current_enc1 - diff, (int)current_enc1, diff, 
                (int)r_count, abs_coord, rot);
        }

        static int16_t l9=0, l10=0;
        if (msg->data[9] != l9 || msg->data[10] != l10) {
            RCLCPP_INFO(get_logger(), "SW Changed! [下(9):%d, 上(10):%d]", 
                        msg->data[9], msg->data[10]);
            l9 = msg->data[9]; l10 = msg->data[10];
        }

        // 以降、受信データを使った処理を記述

        // 受信データ処理ここまで
    }

    uint8_t device_id_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<int16_t> data_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet R2_MotionCtrl";
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

    auto hardware_control = std::make_shared<HardWareControl>(TARGET_DEVICE_ID);
    exec.add_node(hardware_control);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
