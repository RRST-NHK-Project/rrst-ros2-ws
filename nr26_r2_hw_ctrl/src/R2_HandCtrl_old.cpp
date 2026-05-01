/*
R2ハンド制御
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

// まだ未確認！！！
// 壊しても筆者は責任を負いません！！
// リーダブルコードを心がけていますが、変数名や関数名が分かりにくい場合は遠慮なく質問してください。
// ラムダ式を多用しています。

// 標準
#include <chrono> // 時間管理
#include <cstdlib>
#include <iostream>
#include <memory> //ポインタ用
#include <thread>
#include <vector> //動的配列 std::vector を使うため

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>

// 以下マイコンに合わせて設定
#define OUTPUT_DEVICE_ID 2 // 出力マイコン（モーター制御）のID
#define INPUT_DEVICE_ID 3  // 入力マイコン（マイクロスイッチやエンコーダ）のID
#define TX16NUM 24         // 送信データ数
#define RX16NUM 17         // 受信データ数
#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

using namespace std::chrono_literals;

class serial_tx_7 : public rclcpp::Node {
    // rclcpp::Nodeを継承してノードを作成
public:
    enum HandState {
        // ハンドの状態定義
        HOME,   // 初期位置
        READY,  // 回収待機
        UP,     // 上段回収
        MIDDLE, // 中段回収
        LOW,    // 下段回収
        HOLD,   // KFS保持
        MOVING, // KFS移動
        SHOOT   // TR中段シュート
    };

    enum TargetHeight {
        // KFSがある段の高さ定義
        HIGH        // HIGH
            MIDDLE, // MIDDLE
        LOW,        // LOW
    };

    /*
    マイコンに送信される配列"data_"の解析
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

    serial_tx_7() : Node("serial_tx_7_") {

        // 送信データを初期化
        tx_data_.assign(25, 0);

        // GUIからノードを受け取りたい（未実装）
        this->command_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "/serial_tx_7", 10,
            [this](const std_msgs::msg::Int8::SharedPtr msg) {
                this->on_command_received(msg);
            });

        // マイコンへのパブリッシャー
        publisher_ = create_publisher<std_msgs::msg::Int16MultiArray>("serial_tx_7", 10);

        // 50ms周期で送信タイマーを回す（番兵用）(C++だとselfの代わりにthisポインタを第一引数に使う)
        timer_ = create_wall_timer(50ms, [this]() {
            on_timer_tick();
        });

        RCLCPP_INFO(get_logger(), "R2 Hand Control Node Started.");
    }

private:
    // --- 各動作の数値設定 ---
    void set_home_values() {
        tx_data_[9] = 0;  // サーボ：待機
        tx_data_[17] = 0; // シリンダー：縮める
        tx_data_[18] = 0; // ハンド：開く
    }
    void set_ready_values() {
        if (target_height_ == TargetHeight::HIGH) {
            tx_data_[9] = 45; // 高い段差用角度
        } else {
            tx_data_[9] = 90; // 低い段差用角度
        }
        tx_data_[17] = 1; // シリンダー：伸ばす
    }
    void set_up_values() {
        //上回収：3度
        tx_data_[18] = 1; // ハンド：閉じる
    }

    void set_middle_values() {
        tx_data_[18] = 1; // ハンド：閉じる
    }

    void set_low_values() {
        tx_data_[18] = 1; // ハンド：閉じる
    }
    void set_hold_values() {
        tx_data_[17] = 1; // シリンダー：伸ばす
        tx_data_[18] = 1; // ハンド：閉じる
    }
    void set_moving_values() {
        //110,117
        tx_data_[17] = 1; // シリンダー：伸ばす
        tx_data_[18] = 1; // ハンド：閉じる
    }
    void set_shoot_values() {
        tx_data_[17] = 1; // シリンダー：伸ばす
        tx_data_[18] = 1; // ハンド：閉じる
    }

    void on_command_received(const std_msgs::msg::Int8::SharedPtr msg) {
        int cmd = msg->data;
        // 状態遷移に応じて設定
        if (cmd == 10) {
            target_height_ = TargetHeight::LOW;
            current_state_ = HandState::EXTEND;
        } else if (cmd == 20) {
            target_height_ = TargetHeight::HIGH;
            current_state_ = HandState::EXTEND;
        } else if (cmd == 0) {
            current_state_ = HandState::HOME;
        }
        state_duration_count_ = 0; // カウンタリセット
    }

    void on_timer_tick() {
        // シーケンスの更新
        update_sequence();

        // データの送信
        std_msgs::msg::Int16MultiArray msg;
        msg.data = tx_data_;
        publisher_->publish(msg);

        // 状態維持時間をカウント
        state_duration_count_++;
    }

    void update_sequence() {
        switch (current_state_) {
        case HandState::HOME:
            set_home_values();
            break;

        case HandState::EXTEND:
            set_extend_values();
            // 1秒待機
            if (state_duration_count_ > 20) {
                current_state_ = HandState::GRASP;
                state_duration_count_ = 0;
            }
            break;

        case HandState::GRASP:
            set_grasp_values();
            // 0.5秒待機
            if (state_duration_count_ > 10) {
                current_state_ = HandState::RETRACT;
                state_duration_count_ = 0;
            }
            break;

        case HandState::RETRACT:
            tx_data_[17] = 0; // ハンドを閉じたままシリンダーを戻す
            // 1秒待機してHOMEへ
            if (state_duration_count_ > 20) {
                current_state_ = HandState::HOME;
                state_duration_count_ = 0;
            }
            break;
        }
    }

    std::vector<int16_t> tx_data_; // マイコンへ送信するデータを格納するベクター（可変長配列用）
    HandState current_state_ = HandState::HOME;
    TargetHeight target_height_ = TargetHeight::LOW;
    int state_duration_count_ = 0;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr command_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);                                 // ROS2の初期化
    auto hand_control_node = std::make_shared<serial_tx_7>(); // ノードのインスタンス作成
    rclcpp::spin(hand_control_node);                          // ノードをスピンしてコールバックを処理
    rclcpp::shutdown();                                       // 処理終了

    return 0;
}