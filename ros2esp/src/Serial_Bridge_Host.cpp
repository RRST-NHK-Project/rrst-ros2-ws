/*
Serial_Bridgeノードのホスト側プログラム
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>

// 以下マイコンに合わせて設定
#define TARGET_DEVICE_ID 2 // 宛先マイコンのID
#define TX16NUM 24         // 送信データ数
#define RX16NUM 17         // 受信データ数

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

class DriverInterface : public rclcpp::Node {
public:
    DriverInterface(uint8_t device_id)
        : Node("driver_interface_" + std::to_string(device_id)),
          device_id_(device_id) {

        // 配列を0で初期化
        data_.assign(TX16NUM, 0);

        // joyノードのSubscribe
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&DriverInterface::ps4_listener_callback, this, std::placeholders::_1));

        // seial_bridgeへpublish
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(device_id_), 10);

        // timer_callbackを呼び出すタイマーを作成
        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&DriverInterface::publisher_timer_callback, this));

        sensor_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(device_id_),
            10,
            std::bind(&DriverInterface::sensor_callback,
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
        float RS_Y = msg->axes[4];

        bool CROSS = msg->buttons[0];
        bool CIRCLE = msg->buttons[1];
        bool TRIANGLE = msg->buttons[2];
        // bool SQUARE = msg->buttons[3];

        bool LEFT = msg->axes[6] == 1.0;
        bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        bool L1 = msg->buttons[4];
        bool R1 = msg->buttons[5];

        float L2_DIGITAL = (-1 * msg->axes[2] + 1) / 2;
        float R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2;

        bool L2 = msg->buttons[6];
        bool R2 = msg->buttons[7];

        bool SHARE = msg->buttons[8];
        bool OPTION = msg->buttons[9];
        bool PS = msg->buttons[10];

        bool L3 = msg->buttons[11];
        bool R3 = msg->buttons[12];

        // 以降、配列data_を操作する

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

        int16_t ENC1 = msg->data[1];
        int16_t ENC2 = msg->data[2];
        int16_t ENC3 = msg->data[3];
        int16_t ENC4 = msg->data[4];
        int16_t ENC5 = msg->data[5];
        int16_t ENC6 = msg->data[6];
        int16_t ENC7 = msg->data[7];
        int16_t ENC8 = msg->data[8];

        int16_t SW1 = msg->data[9];
        int16_t SW2 = msg->data[10];
        int16_t SW3 = msg->data[11];
        int16_t SW4 = msg->data[12];
        int16_t SW5 = msg->data[13];
        int16_t SW6 = msg->data[14];
        int16_t SW7 = msg->data[15];
        int16_t SW8 = msg->data[16];

        // 以降、受信データを使った処理を記述

        // 受信データ処理ここまで
    }

    uint8_t device_id_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<int16_t> data_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;

    auto driver_interface = std::make_shared<DriverInterface>(TARGET_DEVICE_ID);
    exec.add_node(driver_interface);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}