/*
RRST-NHK-Project 2025
PID on ROS 2, Work In Progress
生成AIコードの挿入禁止
*/

// 標準
#include <algorithm> // std::clamp
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

// 自作クラス
#include "include/IP.hpp"
#include "include/UDP.hpp"

#define MC_PRINTF 0 // マイコン側のprintfを無効化・有効化(0 or 1)

#define MAX_OUTPUT 30.0 // PIDの出力上限

std::vector<int16_t> data(19, 0); // マイコンに送信される配列"data"
std::vector<float_t> deg(4, 0.0); // マイコンに送信される配列"data"

std::vector<float_t> target(4, 0.0); // マイコンに送信される配列"data"

float Kp = 0.01;
float Ki = 0.0;
float Kd = 0.0;
/*
マイコンに送信される配列"data"
debug: マイコンのprintfを有効化, MD: モータードライバー, TR: トランジスタ
| data[n] | 詳細 | 範囲 |
| ---- | ---- | ---- |
| data[0] | debug | 0 or 1 |
| data[1] | MD1 | -100 ~ 100 |
| data[2] | MD2 | -100 ~ 100 |
| data[3] | MD3 | -100 ~ 100 |
| data[4] | MD4 | -100 ~ 100 |
| data[5] | MD5 | -100 ~ 100 |
| data[6] | MD6 | -100 ~ 100 |
| data[7] | Servo1 | 0 ~ 270 |
| data[8] | Servo2 | 0 ~ 270 |
| data[9] | Servo3 | 0 ~ 270 |
| data[10] | Servo4 | 0 ~ 270 |
| data[11] | TR1 | 0 or 1|
| data[12] | TR2 | 0 or 1|
| data[13] | TR3 | 0 or 1|
| data[14] | TR4 | 0 or 1|
| data[15] | TR5 | 0 or 1|
| data[16] | TR6 | 0 or 1|
| data[17] | TR7 | 0 or 1|
| data[18] | TR8 | 0 or 1|
*/

class ENC_Listener : public rclcpp::Node {
public:
    ENC_Listener()
        : Node("enc_listener") {
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "enc", 10,
            std::bind(&ENC_Listener::enc_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "ENC Listener initialized");
    }

private:
    void enc_listener_callback(
        const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        deg[0] = msg->data[0];
        deg[1] = msg->data[1];
        deg[2] = msg->data[2];
        deg[3] = msg->data[3];
        // std::cout << deg[0] << ", ";
        // std::cout << deg[1] << ", ";
        // std::cout << deg[2] << ", ";
        // std::cout << deg[3] << std::endl;
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

class PID_Params_Listener : public rclcpp::Node {
public:
    PID_Params_Listener() : Node("pid_params_listener") {
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "pid_params", 10,
            std::bind(&PID_Params_Listener::pid_params_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),

                    "PID Parameter Listener initialized");
    }

private:
    void pid_params_listener_callback(
        const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        Kp = msg->data[0];
        Ki = msg->data[1];
        Kd = msg->data[2];
        target[2] = msg->data[3];
        std::cout << Kp << ", ";
        std::cout << Ki << ", ";
        std::cout << Kd << ", ";
        std::cout << target[2] << std::endl;
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

class PID : public rclcpp::Node {
public:
    PID(const std::string &ip, int port)
        : Node("pid"), udp_(ip, port) {
        // タイマーの作成：50msごとにPID制御を実行
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PID::timer_callback, this));
    }

private:
    void timer_callback() {

        float dt = 0.05 / 1000000000;  // PID制御の周期（秒）
        float Error[4] = {0.0};        // 誤差
        float Integral[4] = {0.0};     // 積分項
        float Differential[4] = {0.0}; // 微分項
        float last_Error[4] = {0.0};   // 前回の誤差
        float output[4] = {0.0};       // PID出力

        //---------------------------PID---------------------------//

        for (int i = 0; i <= 3; i++) {
            Error[i] = target[i] - deg[i];                        // P
            Integral[i] += ((Error[i] + last_Error[i]) * dt / 2); // I
            Differential[i] = (Error[i] - last_Error[i]) / dt;    // D

            last_Error[i] = Error[i];
            output[i] = ((Kp * Error[i]) + (Ki * Integral[i]) +
                         (Kd * Differential[i]));             // PID
            output[i] = std::clamp(output[i], -50.0f, 50.0f); //-50 ~ 50に収める
            data[i + 1] = (int)output[i];
        }
        // std::cout << output[0] << ", ";
        // std::cout << output[1] << ", ";
        // std::cout << output[2] << ", ";
        // std::cout << output[3] << std::endl;

        // std::cout << data[1] << ", ";
        // std::cout << data[2] << ", ";
        // std::cout << data[3] << ", ";
        // std::cout << data[4] << std::endl;
        udp_.send(data);

        // std::cout << Differential[3] << std::endl;
        //---------------------------PID---------------------------//
    }
    rclcpp::TimerBase::SharedPtr timer_; // タイマーのポインタ
    UDP udp_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;
    auto enc_listener = std::make_shared<ENC_Listener>();
    auto pid = std::make_shared<PID>(IP_DR_SD, PORT_DR_SD);
    auto pid_params_listener = std::make_shared<PID_Params_Listener>();
    exec.add_node(enc_listener);
    exec.add_node(pid);
    exec.add_node(pid_params_listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}