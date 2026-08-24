/*
ros2can (CANホスト) ノードのホスト側プログラム
Serial_Bridge_Host.cpp のros2can版。Ros2CanPacketControllerを使用してノード/スロット
分配を意識せずに送受信配列へアクセスする。
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

// ライブラリ
#include "common/Ros2CanPacketController.hpp"

// 以下マイコンに合わせて設定
// TX/RX_DEVICE_ID は CANホスト自身のシリアルフレームDEVICE_ID
// (firmware/xiao-esp32-s3_can2io/src/config.hpp の DEVICE_ID)。
// CANバス上の各ノードのCAN_ID (101,102,103,104) とは別物なので注意。
#define TX_DEVICE_ID 1 // 送信先CANホストのDEVICE_ID
#define RX_DEVICE_ID 1 // 受信先CANホストのDEVICE_ID

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

// スティックのデッドゾーン
#define DEADZONE_L 0.3
#define DEADZONE_R 0.3

class Ros2CanHostControl : public rclcpp::Node {
public:
    Ros2CanHostControl(uint8_t tx_device_id, uint8_t rx_device_id)
        : Node("ros2can_host_" + std::to_string(tx_device_id)),
          tx_device_id_(tx_device_id),
          rx_device_id_(rx_device_id) {

        /*
        ros2canが送受信する配列(Ros2CanPacketController::tx_ / rx_)は
        serial_bridge互換の24 x int16スロットだが、CANバス上の最大
        Ros2CanPacketController::NODE_COUNT台のノードへ
        Ros2CanPacketController::SLOTS_PER_NODE ずつ分配される。
        実機はDCモータ非搭載 (ENCx2, SWx3, SERVOx3のみ)。

        | ノードローカルslot | 指令(TX) | 帰還(RX) |
        | ---- | ---- | ---- |
        | 0 | SERVO1 (0~270) | SW1 |
        | 1 | SERVO2 (0~270) | SW2 |
        | 2 | SERVO3 (0~270) | SW3 |
        | 3 | 予備 | ENC1 |
        | 4 | 予備 | ENC2 |

        SERVOn と SWn はピン共有 (ファームウェア config.hpp の MULTIn で切替)。
        グローバルslot index = node(0-origin) * SLOTS_PER_NODE + local_index
        ノードのCAN_ID = Ros2CanPacketController::canId(node) (101,102,103,104)
        */

        // joyノードのSubscribe
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&Ros2CanHostControl::ps4_listener_callback, this, std::placeholders::_1));

        // ros2canへpublish
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(tx_device_id_), 10);

        // timer_callbackを呼び出すタイマーを作成
        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&Ros2CanHostControl::publisher_timer_callback, this));

        sensor_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(rx_device_id_),
            10,
            std::bind(&Ros2CanHostControl::sensor_callback,
                      this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "serial_tx_%d started.", tx_device_id_);
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
        // bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        // bool UP = msg->axes[7] == 1.0;
        // bool DOWN = msg->axes[7] == -1.0;

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

        // 以降、Ros2CanPacketControllerを操作する
        // 例: LS_Yでノード1(CAN_ID=101)のSERVO1を0~270度に割り当てる場合
        // ctrlPkt_.setServo(0, 1, static_cast<int>((LS_Y + 1.0) / 2.0 * 270));

        if (CROSS) {
            ctrlPkt_.setServo(0, 1, 0);
        } else if (CIRCLE) {
            ctrlPkt_.setServo(0, 1, 270);
        } else {
            ctrlPkt_.setServo(0, 1, 135);
        }

        // デバッグ用
        // RCLCPP_INFO(
        //     get_logger(),
        //     "node0 servo=[%d,%d,%d]",
        //     ctrlPkt_.tx_[0], ctrlPkt_.tx_[1], ctrlPkt_.tx_[2]);

        // 配列操作ここまで
    }

    // publish
    void publisher_timer_callback() {
        std_msgs::msg::Int16MultiArray msg;

        msg.data = ctrlPkt_.toVector();

        publisher_->publish(msg);
    }

    void
    sensor_callback(
        const std_msgs::msg::Int16MultiArray::SharedPtr msg) {

        ctrlPkt_.updateRx(msg->data);

        // bool SW1 = ctrlPkt_.getSW(0, 1);
        // bool SW2 = ctrlPkt_.getSW(0, 2);
        // bool SW3 = ctrlPkt_.getSW(0, 3);

        // int16_t ENC1 = ctrlPkt_.getEnc(0, 1);
        // int16_t ENC2 = ctrlPkt_.getEnc(0, 2);

        // 以降、受信データを使った処理を記述

        // 受信データ処理ここまで
    }

    uint8_t tx_device_id_;
    uint8_t rx_device_id_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    Ros2CanPacketController ctrlPkt_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet Ros2Can Host";
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

    auto ros2can_host = std::make_shared<Ros2CanHostControl>(TX_DEVICE_ID, RX_DEVICE_ID);
    exec.add_node(ros2can_host);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
