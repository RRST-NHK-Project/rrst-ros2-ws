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

// 以下マイコンに合わせて設定
#define TARGET_DEVICE_ID 2 // 宛先マイコンのID
#define TX16NUM 24         // 送信データ数
#define RX16NUM 17         // 受信データ数

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

// スティックのデッドゾーン
#define DEADZONE_L 0.3
#define DEADZONE_R 0.3

bool CHANGEMODE = false;
bool REVERSEMODE = false;
bool YAWMODE = false;

// グローバル変数
int deg = 0;
int truedeg = 0;

// 速度
int wheelspeed = 64;
int yawspeed = 32;

// サーボの組み付け時のズレを補正（度数法）
int SERVO1_CAL = 0;
int SERVO2_CAL = 0;
int SERVO3_CAL = 0;
int SERVO4_CAL = 0;

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

        float LS_X = -1 * msg->axes[0];
        float LS_Y = msg->axes[1];
        float RS_X = -1 * msg->axes[3];
        // float RS_Y = msg->axes[4];

        bool CROSS = msg->buttons[0];
        // bool CIRCLE = msg->buttons[1];
        // bool TRIANGLE = msg->buttons[2];
        // bool SQUARE = msg->buttons[3];

        bool LEFT = msg->axes[6] == 1.0;
        bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        // bool L1 = msg->buttons[4];
        // bool R1 = msg->buttons[5];

        // float L2 = (-1 * msg->axes[2] + 1) / 2;
        float R2 = (-1 * msg->axes[5] + 1) / 2;

        bool SHARE = msg->buttons[8];
        bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        bool R3 = msg->buttons[12];

        static bool last_option = false; // 前回の状態を保持する static 変数
        static bool last_share = false;

        // OPTION のラッチ状態を保持する static 変数（初期状態は OFF とする）
        static bool option_latch = false;
        static bool share_latch = false;

        static bool last_R3 = false;
        static bool R3_latch = false;

        // 以降、配列data_を操作する

        float rad = atan2(LS_Y, LS_X);
        deg = rad * 180 / M_PI;
        if (OPTION && !last_option) {
            option_latch = !option_latch;
        }
        if (SHARE && !last_share) {
            share_latch = !share_latch;
            // Automation::auto_turn(udp_);
        }
        if (R3 && !last_R3) {
            R3_latch = !R3_latch;
        }

        // ！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
        // もとの移動方法！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
        // ！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！

        last_share = SHARE;
        REVERSEMODE = share_latch;
        last_option = OPTION;
        CHANGEMODE = option_latch;
        last_R3 = R3;
        YAWMODE = R3_latch;

        // XY座標での正しい角度truedeg

        if (REVERSEMODE == 0) {
            truedeg = deg;
            if ((0 <= truedeg) && (truedeg <= 180)) {
                truedeg = truedeg;
            }
            if ((-180 <= truedeg) && (truedeg <= 0)) {
                truedeg = -truedeg + 360;
            }

            // ！！！！！最重要！！！！！
            //  XY座標での９０度の位置に１３５度を変換して計算
            if ((-180 <= deg) && (deg <= -135)) {
                deg = -deg - 135;
            } else {
                deg = 225 - deg;
            }

            // deadzone追加
            if ((fabs(LS_X) <= DEADZONE_R) && (fabs(LS_Y) <= DEADZONE_R) &&
                (fabs(RS_X) <= DEADZONE_L)) {
                deg = 135;
                data_[1] = 0;
                data_[2] = 0;
                data_[3] = 0;
                data_[4] = 0;
                data_[9] = deg + SERVO1_CAL;
                data_[10] = deg + SERVO2_CAL;
                data_[11] = deg + SERVO3_CAL;
                data_[12] = deg + SERVO4_CAL;
            }

            data_[1] = -wheelspeed * R2;
            data_[2] = -wheelspeed * R2;
            data_[3] = -wheelspeed * R2;
            data_[4] = -wheelspeed * R2;
            data_[9] = deg + SERVO1_CAL;
            data_[10] = deg + SERVO2_CAL;
            data_[11] = deg + SERVO3_CAL;
            data_[12] = deg + SERVO4_CAL;

            if (LEFT) {
                deg = 45;
                data_[1] = -wheelspeed * R2;
                data_[2] = -wheelspeed * R2;
                data_[3] = -wheelspeed * R2;
                data_[4] = -wheelspeed * R2;
            }
            if (RIGHT) {
                deg = 45;
                data_[1] = wheelspeed * R2;
                data_[2] = wheelspeed * R2;
                data_[3] = wheelspeed * R2;
                data_[4] = wheelspeed * R2;
            }
            if (UP) {
                deg = 135;
                data_[1] = -wheelspeed * R2;
                data_[2] = -wheelspeed * R2;
                data_[3] = -wheelspeed * R2;
                data_[4] = -wheelspeed * R2;
            }
            if (DOWN) {
                deg = 135;
                data_[1] = wheelspeed * R2;
                data_[2] = wheelspeed * R2;
                data_[3] = wheelspeed * R2;
                data_[4] = wheelspeed * R2;
            }

            // 独ステが扱えない範囲の変換
            if ((270 < deg) && (deg < 360)) {
                deg = deg - 180;
                data_[1] = wheelspeed * R2;
                data_[2] = wheelspeed * R2;
                data_[3] = wheelspeed * R2;
                data_[4] = wheelspeed * R2;
                data_[9] = deg + SERVO1_CAL;
                data_[10] = deg + SERVO2_CAL;
                data_[11] = deg + SERVO3_CAL;
                data_[12] = deg + SERVO4_CAL;
            }
            // 角度だけYAW
            if (R3_latch == 0) {
                data_[9] = deg + SERVO1_CAL;
                data_[10] = deg + SERVO2_CAL;
                data_[11] = deg + SERVO3_CAL;
                data_[12] = deg + SERVO4_CAL;
            }
            if (R3_latch == 1) {
                data_[9] = 180 + SERVO1_CAL;
                data_[10] = 90 + SERVO2_CAL;
                data_[11] = 90 + SERVO3_CAL;
                data_[12] = 180 + SERVO4_CAL;
            }
            // 時計回りYAW回転
            if (RS_X < 0 && fabs(RS_X) >= DEADZONE_R) {
                data_[9] = 180 + SERVO1_CAL;
                data_[10] = 90 + SERVO2_CAL;
                data_[11] = 90 + SERVO3_CAL;
                data_[12] = 180 + SERVO4_CAL;
                data_[1] = -yawspeed;
                data_[2] = yawspeed;
                data_[3] = -yawspeed;
                data_[4] = yawspeed;
            }
            // 半時計回りYAW回転
            if (0 < RS_X && fabs(RS_X) >= DEADZONE_R) {
                data_[9] = 180 + SERVO1_CAL;
                data_[10] = 90 + SERVO2_CAL;
                data_[11] = 90 + SERVO3_CAL;
                data_[12] = 180 + SERVO4_CAL;
                data_[1] = yawspeed;
                data_[2] = -yawspeed;
                data_[3] = yawspeed;
                data_[4] = -yawspeed;
            }
        }
        // 反転モード
        //
        if (REVERSEMODE == 1) {
            truedeg = deg;
            if ((0 <= truedeg) && (truedeg <= 180)) {
                truedeg = truedeg;
            }
            if ((-180 <= truedeg) && (truedeg <= 0)) {
                truedeg = -truedeg + 360;
            }

            // ！！！！！最重要！！！！！
            //  XY座標での９０度の位置に１３５度を変換して計算
            if ((-180 <= deg) && (deg <= -135)) {
                deg = -deg - 135;
            } else {
                deg = 225 - deg;
            }

            // deadzone追加
            if ((fabs(LS_X) <= DEADZONE_R) && (fabs(LS_Y) <= DEADZONE_R) &&
                (fabs(RS_X) <= DEADZONE_L)) {
                deg = 135;
                data_[1] = 0;
                data_[2] = 0;
                data_[3] = 0;
                data_[4] = 0;
                data_[9] = deg + SERVO1_CAL;
                data_[10] = deg + SERVO2_CAL;
                data_[11] = deg + SERVO3_CAL;
                data_[12] = deg + SERVO4_CAL;
            }

            data_[1] = wheelspeed * R2;
            data_[2] = wheelspeed * R2;
            data_[3] = wheelspeed * R2;
            data_[4] = wheelspeed * R2;
            data_[9] = deg + SERVO1_CAL;
            data_[10] = deg + SERVO2_CAL;
            data_[11] = deg + SERVO3_CAL;
            data_[12] = deg + SERVO4_CAL;

            if (LEFT) {
                deg = 45;
                data_[1] = wheelspeed * R2;
                data_[2] = wheelspeed * R2;
                data_[3] = wheelspeed * R2;
                data_[4] = wheelspeed * R2;
            }
            if (RIGHT) {
                deg = 45;
                data_[1] = -wheelspeed * R2;
                data_[2] = -wheelspeed * R2;
                data_[3] = -wheelspeed * R2;
                data_[4] = -wheelspeed * R2;
            }
            if (UP) {
                deg = 135;
                data_[1] = wheelspeed * R2;
                data_[2] = wheelspeed * R2;
                data_[3] = wheelspeed * R2;
                data_[4] = wheelspeed * R2;
            }
            if (DOWN) {
                deg = 135;
                data_[1] = -wheelspeed * R2;
                data_[2] = -wheelspeed * R2;
                data_[3] = -wheelspeed * R2;
                data_[4] = -wheelspeed * R2;
            }

            // 射出直前にサーボを直角に向けストップ
            if (CROSS) {
                data_[1] = 0;
                data_[2] = 0;
                data_[3] = 0;
                data_[4] = 0;
                data_[5] = 0;
                data_[6] = 0;
                data_[9] = 45 + SERVO1_CAL;
                data_[10] = 45 + SERVO2_CAL;
                data_[11] = 45 + SERVO3_CAL;
                data_[12] = 45 + SERVO4_CAL;
            }

            // 独ステが扱えない範囲の変換
            if ((270 < deg) && (deg < 360)) {
                deg = deg - 180;
                data_[1] = -wheelspeed * R2;
                data_[2] = -wheelspeed * R2;
                data_[3] = -wheelspeed * R2;
                data_[4] = -wheelspeed * R2;
                data_[9] = deg + SERVO1_CAL;
                data_[10] = deg + SERVO2_CAL;
                data_[11] = deg + SERVO3_CAL;
                data_[12] = deg + SERVO4_CAL;
            }
            // 角度だけYAW
            if (R3_latch == 0) {
                data_[9] = deg + SERVO1_CAL;
                data_[10] = deg + SERVO2_CAL;
                data_[11] = deg + SERVO3_CAL;
                data_[12] = deg + SERVO4_CAL;
            }
            if (R3_latch == 1) {
                data_[9] = 180 + SERVO1_CAL;
                data_[10] = 90 + SERVO2_CAL;
                data_[11] = 90 + SERVO3_CAL;
                data_[12] = 180 + SERVO4_CAL;
            }
            // 時計回りYAW回転
            if (RS_X < 0 && fabs(RS_X) >= DEADZONE_R) {
                data_[9] = 180 + SERVO1_CAL;
                data_[10] = 90 + SERVO2_CAL;
                data_[11] = 90 + SERVO3_CAL;
                data_[12] = 180 + SERVO4_CAL;
                data_[1] = -yawspeed;
                data_[2] = yawspeed;
                data_[3] = -yawspeed;
                data_[4] = yawspeed;
            }
            // 半時計回りYAW回転
            if (0 < RS_X && fabs(RS_X) >= DEADZONE_R) {
                data_[9] = 180 + SERVO1_CAL;
                data_[10] = 90 + SERVO2_CAL;
                data_[11] = 90 + SERVO3_CAL;
                data_[12] = 180 + SERVO4_CAL;
                data_[1] = yawspeed;
                data_[2] = -yawspeed;
                data_[3] = yawspeed;
                data_[4] = -yawspeed;
            }
        }

        if (SHARE) {
            // Automation::auto_turn(udp_);
        }
        // std::cout << REVERSEMODE << std::endl;

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

    rclcpp::executors::MultiThreadedExecutor exec;

    auto driver_interface = std::make_shared<HardWareControl>(TARGET_DEVICE_ID);
    exec.add_node(driver_interface);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
