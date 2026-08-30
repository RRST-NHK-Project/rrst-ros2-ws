/*
ソウキハンド機構制御
ダイヤフラムポンプとサーボ2つをPS4コントローラで動かす。
ポンプは押している間だけ一定の出力で回し、離すと停止する。
サーボは押すごとに0度と90度を交互に切り替える。

マイコンへ送る 24 x int16 の配列を直接組み立てる。各指令を載せるスロットは
MOTOR_SLOT / SERVO1_SLOT / SERVO2_SLOT で指定する。

操作割り当て:
  | 入力       | 対象   | 動作                        |
  | ---------- | ------ | --------------------------- |
  | R1         | ポンプ | 運転 (離すと停止)           |
  | △ TRIANGLE | サーボ1 | 押すごとに 0度 ⇔ 90度      |
  | × CROSS    | サーボ2 | 押すごとに 0度 ⇔ 90度      |

Copyright (c) 2026 RRST-NHK-Project. All rights reserved.
*/

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>

// 以下マイコンに合わせて設定
#define TARGET_DEVICE_ID 101 // 宛先マイコンのID
#define TX16NUM 24           // 送信データ数
#define RX16NUM 24           // 受信データ数

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

// 送信フレーム上で各指令を載せるスロット(0-origin)。
// 使用するマイコンのスロット割り当てに合わせて変更すること。
#define MOTOR_SLOT 0
#define SERVO1_SLOT 1
#define SERVO2_SLOT 2

// 押している間の一定出力。単位と範囲はモータドライバの仕様に合わせること
// (例: duty比なら0~100、生rpm指令ならrpm値)。
#define MOTOR_OUTPUT 100

// サーボが交互に取る2つの角度[deg]
#define SERVO_ANGLE_OFF 0
#define SERVO_ANGLE_ON 90

// スティックのデッドゾーン
#define DEADZONE_L 0.3
#define DEADZONE_R 0.3

class HardWareControl : public rclcpp::Node {
public:
    HardWareControl(uint8_t device_id)
        : Node("hardware_control_" + std::to_string(device_id)),
          device_id_(device_id) {

        // 送信配列を0で初期化
        tx_.fill(0);
        /*
        マイコンに送信される配列 tx_ (24 x int16)
        本ノードが使うのは MOTOR_SLOT / SERVO1_SLOT / SERVO2_SLOT だけで、
        残りのスロットは0のまま送る。他のアクチュエータを併用する場合は、
        そのスロット番号を追加で定義して同じ配列へ書き込むこと。
        */

        // サーボ初期値 (起動時は OFF 側の角度)
        tx_[SERVO1_SLOT] = SERVO_ANGLE_OFF;
        tx_[SERVO2_SLOT] = SERVO_ANGLE_OFF;

        // joyノードのSubscribe
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&HardWareControl::ps4_listener_callback, this, std::placeholders::_1));

        // マイコンへpublish
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

        // マイコン側にシリアル途絶時のフェイルセーフは無く、最後に受信した指令を
        // 保持し続ける。ノード終了時はポンプを止める。
        rclcpp::on_shutdown([this]() { send_zero_and_stop(); });

        RCLCPP_INFO(get_logger(),
                    "serial_tx_%d started.", device_id_);
    }

private:
    // 手動操作の入力状態（joyコールバックが書き、タイマーコールバックが読む）
    std::atomic<bool> motor_on_{false};    // R1: ポンプ運転
    std::atomic<bool> servo1_state_{false}; // △: false=0度, true=90度
    std::atomic<bool> servo2_state_{false}; // ×: false=0度, true=90度

    // =====================================================================
    // PS4コントローラーコールバック（手動制御用）
    // =====================================================================
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        // float LS_X = -1 * msg->axes[0];
        // float LS_Y = msg->axes[1];
        // float RS_X = -1 * msg->axes[3];
        // float RS_Y = msg->axes[4];

        bool CROSS = msg->buttons[0];
        // bool CIRCLE = msg->buttons[1];
        bool TRIANGLE = msg->buttons[2];
        // bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        // bool UP = msg->axes[7] == 1.0;
        // bool DOWN = msg->axes[7] == -1.0;

        // bool L1 = msg->buttons[4];
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

        // 以降、配列tx_を操作する

        // =================================================================
        // R1:「ポンプの運転」
        // 押している間だけ一定出力(MOTOR_OUTPUT)で回し、離すと停止する。
        // =================================================================
        motor_on_.store(R1);

        // =================================================================
        // △,×:「サーボの切り替え」
        // 押すごとに SERVO_ANGLE_OFF と SERVO_ANGLE_ON を交互に取る。
        // 立ち上がりエッジでのみ反転させるので、押しっぱなしにしても
        // 1回の押下につき1回だけ切り替わる。
        // =================================================================
        static bool last_triangle = false;
        if (TRIANGLE && !last_triangle) {
            servo1_state_.store(!servo1_state_.load());
        }
        last_triangle = TRIANGLE;

        static bool last_cross = false;
        if (CROSS && !last_cross) {
            servo2_state_.store(!servo2_state_.load());
        }
        last_cross = CROSS;

        // 配列操作ここまで
    }

    // publish
    void publisher_timer_callback() {
        std_msgs::msg::Int16MultiArray msg;

        // ★★★ 手動操作の指令値を毎周期適用 ★★★
        tx_[MOTOR_SLOT] = motor_on_.load() ? MOTOR_OUTPUT : 0;
        tx_[SERVO1_SLOT] = servo1_state_.load() ? SERVO_ANGLE_ON : SERVO_ANGLE_OFF;
        tx_[SERVO2_SLOT] = servo2_state_.load() ? SERVO_ANGLE_ON : SERVO_ANGLE_OFF;

        msg.data = std::vector<int16_t>(tx_.begin(), tx_.end());
        publisher_->publish(msg);
    }

    void
    sensor_callback(
        const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        if (msg->data.size() < RX16NUM) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "serial_rx_%d: data too short (%zu)",
                                 device_id_, msg->data.size());
            return;
        }

        // 以降、受信データを使った処理を記述

        // 受信データ処理ここまで
    }

    // ポンプを止めて送信する (サーボは保持)
    void send_zero_and_stop() {
        motor_on_.store(false);
        publisher_timer_callback();
    }

    uint8_t device_id_;

    // 送信配列本体 (ROS -> マイコン)
    std::array<int16_t, TX16NUM> tx_{};

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet Soki Hand";
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
