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
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

// 自作
#include "PacketController.hpp"
PacketController pkt;

#define TARGET_DEVICE_ID 6
#define PUBLISH_RATE_MS 50

class PIDMecanumController : public rclcpp::Node {
public:
    PIDMecanumController()
        : Node("pid_mecanum_controller") {

        // オドメトリ subscriber (X,Y,Yaw)
        odom_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "odom", 10,
            std::bind(&PIDMecanumController::odom_callback, this, std::placeholders::_1));

        // マイコン送信
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(TARGET_DEVICE_ID), 10);

        // タイマー
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&PIDMecanumController::publish_timer, this));

        // PIDパラメータ
        Kp_x_ = 1.0;
        Ki_x_ = 0.0;
        Kd_x_ = 0.1;
        Kp_y_ = 1.0;
        Ki_y_ = 0.0;
        Kd_y_ = 0.1;

        // 固定目標座標
        target_x_ = 0.3;
        target_y_ = 0.3;

        prev_time_ = this->now();
    }

private:
    // ROS
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // PID
    double Kp_x_, Ki_x_, Kd_x_;
    double Kp_y_, Ki_y_, Kd_y_;
    double integral_x_ = 0.0;
    double integral_y_ = 0.0;
    double prev_error_x_ = 0.0;
    double prev_error_y_ = 0.0;
    rclcpp::Time prev_time_;

    // odom
    double X_ = 0.0, Y_ = 0.0, yaw_ = 0.0;

    // 目標座標
    double target_x_, target_y_;

    // メカナムホイール速度
    float duty_max = 100;
    float v1, v2, v3, v4;

    void odom_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 3)
            return;

        X_ = msg->data[0];
        Y_ = msg->data[1];
        yaw_ = msg->data[2];

        // 誤差計算（オドメトリに合わせて前進F→Y、左右L/R→X）
        double error_x = target_x_ - X_;    // 左右方向
        double error_y = -(target_y_ - Y_); // 前進方向

        // PID制御
        rclcpp::Time now = this->now();
        double dt = (now - prev_time_).seconds();
        prev_time_ = now;

        integral_x_ += error_x * dt;
        integral_y_ += error_y * dt;

        double derivative_x = (error_x - prev_error_x_) / dt;
        double derivative_y = (error_y - prev_error_y_) / dt;

        prev_error_x_ = error_x;
        prev_error_y_ = error_y;

        double vx = Kp_x_ * error_x + Ki_x_ * integral_x_ + Kd_x_ * derivative_x; // 横方向
        double vy = Kp_y_ * error_y + Ki_y_ * integral_y_ + Kd_y_ * derivative_y; // 前後方向
        double wz = 0.0;                                                          // yaw固定

        // 逆運動学（前進Y+, 右X+）
        v1 = vy + vx + wz; // 前左
        v3 = vy - vx - wz; // 前右
        v4 = vy - vx + wz; // 後左
        v2 = vy + vx - wz; // 後右

        // 向き補正
        v3 *= -1; // 前右
        v2 *= -1; // 後右
        // v1 *= -1;
        // v4 *= -1;

        // 正規化
        float max_v = std::max({fabsf(v1), fabsf(v2), fabsf(v3), fabsf(v4)});
        if (max_v < 1.0f)
            max_v = 1.0f;

        v1 /= max_v;
        v2 /= max_v;
        v3 /= max_v;
        v4 /= max_v;

        RCLCPP_INFO(this->get_logger(),
                    "X: %.3f Y: %.3f yaw: %.3f | v1: %.2f v2: %.2f v3: %.2f v4: %.2f",
                    X_, Y_, yaw_, v1, v2, v3, v4);
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