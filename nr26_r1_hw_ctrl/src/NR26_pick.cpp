/*
Serial_Bridgeノードのホスト側プログラム
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

// 以下マイコンに合わせて設定
#define TARGET_DEVICE_ID 2 // 宛先マイコンのID
#define TX16NUM 24         // 送信データ数
#define RX16NUM 17         // 受信データ数

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

// スティックのデッドゾーン
#define DEADZONE_L 0.3
#define DEADZONE_R 0.3

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
        | data[24] | TR8 | 0 or 1|
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
        // bool TRIANGLE = msg->buttons[2];
        bool SQUARE = msg->buttons[3];

        bool LEFT = msg->axes[6] == 1.0;
        bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        // bool L1 = msg->buttons[4];
        // bool R1 = msg->buttons[5];

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
        // CROSS:「マガジン回転」
        // ボタンを一回押すごとにサーボモーターを45°づつ回転する
        // =================================================================

        // data_[1] = CROSS;
        // static int MAG_SERVO_ANGLE[] = {270, 225, 190, 135, 0};
        // static int cross_pre = 0;
        // static int s = 0;
        // static int CROSS_PUSH_MAX = 5;

        // if (CROSS == 1 && cross_pre == 0) {
        //     s = (s + 1) % CROSS_PUSH_MAX;
        // }
        // if (s == 0) {
        //     data_[9] = MAG_SERVO_ANGLE[0];
        // }
        // if (s == 1) {
        //     data_[9] = MAG_SERVO_ANGLE[1];
        // }
        // if (s == 2) {
        //     data_[9] = MAG_SERVO_ANGLE[2];
        // }
        // if (s == 3) {
        //     data_[9] = MAG_SERVO_ANGLE[3];
        // }
        // if (s == 4) {
        //     data_[9] = MAG_SERVO_ANGLE[4];
        // }
        // cross_pre = CROSS;

        // =================================================================
        // CIRCLE:「棒ホールド機構」
        // ボタンを一回押すごとに2つのサーボモーターの角度状態を同時に変化させる
        // =================================================================
        // data_[2] = CIRCLE;
        // angle = 10のとき最下部までお仕込み
        // angle = 245のときマガジンに戻してる

        static int BAR_PUSH_ANGLE = 10;
        static int BAR_HOLD_ANGLE = 43;
        static int BAR_RELE_ANGLE = 245;
        static int circle_pre = 0;
        static int t = 0;
        static int CIRCLE_PUSH_MAX = 3;

        if (CIRCLE == 1 && circle_pre == 0) {
            t = (t + 1) % CIRCLE_PUSH_MAX;
        }
        if (t == 0) {
            data_[10] = BAR_RELE_ANGLE;
        }
        if (t == 1) {
            data_[10] = BAR_HOLD_ANGLE;
        }
        if (t == 2) {
            data_[10] = BAR_PUSH_ANGLE;
        }

        circle_pre = CIRCLE;

        // =================================================================
        // SQUARE:「棒ロック」
        // ボタンを一回押すごとに2つのサーボモーターの角度状態を同時に変化させる
        // =================================================================

        static int BAR_BTM_HOLD_ANGLE = 0;
        static int BAR_BTM_ANGLE = 150;
        static int square_pre = 0;
        static int u = 0;
        static int SQUARE_PUSH_MAX = 2;

        if (SQUARE == 1 && square_pre == 0) {
            u = (u + 1) % SQUARE_PUSH_MAX;
        }
        if (u == 0) {
            data_[11] = BAR_BTM_ANGLE;
        }
        if (u == 1) {
            data_[11] = BAR_BTM_HOLD_ANGLE;
        }

        square_pre = SQUARE;
        // =================================================================
        // UP:「槍押上機構」
        // ボタンを押し続けると槍を押し上げるモーターが回転し続ける
        // =================================================================
        // data_[3] = UP;
        if (UP) {
            data_[1] = 50;
        } else if (DOWN) {
            data_[1] = -50;
        } else {
            data_[1] = 0;
        }
        // =================================================================
        // DOWN:「槍押上機構」
        // ボタンを押し続けると槍を下げるモーターが回転し続ける
        // =================================================================
        // data_[4] = DOWN;

        // }else if (!DOWN){
        //     data_[1] = 0;
        // }

        // =================================================================
        // LR
        // マガジン調整用
        static int MAG_ADJ_STEP = 5;
        static int left_pre = 0;
        static int right_pre = 0;
        static int mag_servo_angle = 270;

        if (LEFT == 1 && left_pre == 0) {
            mag_servo_angle -= MAG_ADJ_STEP;
            data_[9] = mag_servo_angle;
        }
        left_pre = LEFT;
        if (RIGHT == 1 && right_pre == 0) {
            mag_servo_angle += MAG_ADJ_STEP;
            data_[9] = mag_servo_angle;
        }
        right_pre = RIGHT;
        // =================================================================

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
    std::string figletout = "figlet Serial Bridge Host";
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
