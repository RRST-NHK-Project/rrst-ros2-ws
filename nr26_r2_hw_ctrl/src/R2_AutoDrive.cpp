/*
R2 PID メカナム制御ノード
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

// 標準
#include <chrono>
#include <cmath>
#include <iostream>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

// 自作
#include "include/PacketController.hpp"
PacketController pkt;
#include "include/PID.hpp"

#define TARGET_DEVICE_ID 6
#define PUBLISH_RATE_MS 50

class PIDMecanumController : public rclcpp::Node {
public:
    PIDMecanumController()
        : Node("pid_mecanum_controller"),
          pid_x_(1.0, 0.0, 0.0, 1.0),
          pid_y_(1.0, 0.0, 0.0, 1.0) {

        // odom subscriber
        odom_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "odom", 10,
            std::bind(&PIDMecanumController::odom_callback, this, std::placeholders::_1));

        // マイコン送信
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(TARGET_DEVICE_ID), 10);

        // PS4 controller
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&PIDMecanumController::ps4_listener_callback, this, std::placeholders::_1));

        // timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&PIDMecanumController::publish_timer, this));

        prev_time_ = this->now();
    }

private:
    // ROS
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // PID
    PIDController pid_x_;
    PIDController pid_y_;

    rclcpp::Time prev_time_;

    // odom
    float X_ = 0;
    float Y_ = 0;
    float yaw_ = 0;

    // target
    float target_x_;
    float target_y_;

    // mecanum
    float duty_max = 100;
    float v1, v2, v3, v4;

    // odom

    void odom_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 3)
            return;

        X_ = msg->data[0];
        Y_ = msg->data[1];
        yaw_ = msg->data[2];

        rclcpp::Time now = this->now();
        double dt = (now - prev_time_).seconds();
        prev_time_ = now;

        if (dt < 0.001)
            return;

        // PID target
        pid_x_.set_target(target_x_);
        pid_y_.set_target(target_y_);

        // PID output
        float vx = pid_x_.update(X_, dt);
        float vy = -pid_y_.update(Y_, dt);

        float wz = 0.0;

        // メカナム逆運動学
        v1 = vy + vx + wz;
        v3 = vy - vx - wz;
        v4 = vy - vx + wz;
        v2 = vy + vx - wz;

        // 右側モータ反転
        v3 *= -1;
        v2 *= -1;

        // 正規化
        float max_v = std::max({fabs(v1), fabs(v2), fabs(v3), fabs(v4)});
        if (max_v < 1.0)
            max_v = 1.0;

        v1 /= max_v;
        v2 /= max_v;
        v3 /= max_v;
        v4 /= max_v;

        RCLCPP_INFO(this->get_logger(),
                    "X: %.2f Y: %.2f | v1 %.2f v2 %.2f v3 %.2f v4 %.2f",
                    X_, Y_, v1, v2, v3, v4);
    }

    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        float LS_X = -1 * msg->axes[0];
        float LS_Y = msg->axes[1];
        float RS_X = -1 * msg->axes[3];
        float RS_Y = msg->axes[4];

        bool CROSS = msg->buttons[0];
        // bool CIRCLE = msg->buttons[1];
        bool TRIANGLE = msg->buttons[2];
        // bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        bool L1 = msg->buttons[4];
        bool R1 = msg->buttons[5];

        // float L2_DIGITAL = (-1 * msg->axes[2] + 1) / 2;
        float R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2;

        // bool L2 = msg->buttons[6];
        // bool R2 = msg->buttons[7];

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        static bool last_up = false;
        static bool up_latch = false;
        static bool last_down = false;
        static bool down_latch = false;

        if (UP && !last_up) {
            up_latch = !up_latch;
            target_x_ = 30.0;
            target_y_ = 30.0;
        }

        if (DOWN && !last_down) {
            down_latch = !down_latch;
            target_x_ = 0.0;
            target_y_ = 0.0;
        }
        last_up = UP;
        last_down = DOWN;

        // 以降、配列data_を操作する
        float rad = atan2(LS_Y, LS_X);
    }

    void publish_timer() {
        pkt.setMD(MD5, static_cast<int16_t>(v1 * duty_max));
        pkt.setMD(MD6, static_cast<int16_t>(v2 * duty_max));
        pkt.setMD(MD7, static_cast<int16_t>(v3 * duty_max));
        pkt.setMD(MD8, static_cast<int16_t>(v4 * duty_max));

        std_msgs::msg::Int16MultiArray msg;
        msg.data = pkt.toVector();
        publisher_->publish(msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PIDMecanumController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}