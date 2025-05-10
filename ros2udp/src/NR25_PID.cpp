/*
RRST NHK2025
PID on ROS
*/

// 標準
#include <algorithm> // std::clamp を使用するために必要
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

// 自作クラス
#include "include/UDP.hpp"

// IPアドレスとポートの指定
std::string udp_ip = "192.168.8.217"; // 送信先IPアドレス、宛先マイコンで設定したIPv4アドレスを指定
int udp_port = 5000;                  // 送信元ポート番号、宛先マイコンで設定したポート番号を指定

std::vector<int16_t> data(19, 0); // 7~9番を電磁弁制御に転用中（-1 or 1）

float v[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
// float d[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float deg[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

constexpr int NUM_JOINTS = 5; // 制御するジョイント数

// PIDパラメータ
float Kp = 3.5, Ki = 0.0, Kd = 0.02;
float d_target[NUM_JOINTS] = {0.0, 0.0, 0.0, 0.0, 0.0};   // 目標値
float d[NUM_JOINTS] = {0.0, 0.0, 0.0, 0.0, 0.0};          // 現在値
float prev_error[NUM_JOINTS] = {0.0, 0.0, 0.0, 0.0, 0.0}; // 前回の誤差
float integral[NUM_JOINTS] = {0.0, 0.0, 0.0, 0.0, 0.0};   // 積分項
float u[NUM_JOINTS] = {0.0, 0.0, 0.0, 0.0, 0.0};          // PID出力

int control_period_ms = 50; // PID制御の周期（ms）

class ENC_Listener : public rclcpp::Node {
public:
    ENC_Listener(const std::string &ip, int port)
        : Node("nhk25_pid"), udp_(ip, port) {
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "enc_pid", 10,
            std::bind(&ENC_Listener::enc_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "NHK2025 PID");

        // PID制御の実行（100msごとに更新）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(control_period_ms),
            std::bind(&ENC_Listener::pid_control, this));
    }

private:
    void enc_listener_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        v[1] = msg->data[0];
        v[2] = msg->data[1];
        v[3] = msg->data[2];
        v[4] = msg->data[3];
        d[1] = msg->data[4];
        d[2] = msg->data[5];
        d[3] = msg->data[6];
        d[4] = msg->data[7];
        // std::cout << d[1] << " " << d[2] << " " << d[3] << " " << d[4] << std::endl;
    }

    void pid_control() {
        float dt = static_cast<float>(control_period_ms) / 1000.0; // 100msごとに更新

        // **目標角度の更新**

        // for (int i = 1; i < NUM_JOINTS; i++) {
        //     d_target[i] += sin(count_);
        //     // std::cout << "Joint " << i << " target updated to: " << d_target[i] << "°" << std::endl;
        // }

        // count_++;

        d_target[1] = 6.28;

        for (int i = 1; i < NUM_JOINTS; i++) {
            float error = d_target[i] - d[i];                       // 目標値との差
            integral[i] += error * dt;                              // 積分項の計算
            float derivative = (error - prev_error[i]) / dt;        // 微分項の計算
            u[i] = Kp * error + Ki * integral[i] + Kd * derivative; // PID制御計算

            // **制御出力を0~100に制限**
            u[i] = std::clamp(u[i], -100.0f, 100.0f);

            prev_error[i] = error; // 誤差を更新

            std::cout << "Joint " << i << " -> Target: " << d_target[i]
                      << ", Current: " << d[i]
                      << ", PID Output: " << u[i] << std::endl;

            data[1] = u[1];
            udp_.send(data);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    UDP udp_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;
    auto enc_listener = std::make_shared<ENC_Listener>(udp_ip, udp_port);
    exec.add_node(enc_listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
