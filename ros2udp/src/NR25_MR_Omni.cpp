/*
RRST NHK2025
足回り制御
*/

#include <chrono>
#include <math.h>
#include <thread>

#include "include/UDP.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

float duty_max = 70;
float sp_yaw = 0.1;

float deadzone = 0.3; // adjust DS4 deadzone

// IPアドレスとポートの指定
std::string udp_ip = "192.168.8.215"; // 送信先IPアドレス、宛先マイコンで設定したIPv4アドレスを指定
int udp_port = 5000;                  // 送信元ポート番号、宛先マイコンで設定したポート番号を指定

std::vector<int16_t> data(19, 0); // 各モーターの出力（0% ~ 100%）
float v1, v2, v3, v4;

class PS4_Listener : public rclcpp::Node {
public:
    PS4_Listener(const std::string &ip, int port)
        : Node("nhk25_mr_omni"), udp_(ip, port) {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "NHK2025 MR Omni initialized with IP: %s, Port: %d", ip.c_str(),
                    port);
    }

private:
    // コントローラーの入力を取得、使わない入力はコメントアウト推奨

    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        float LS_X = -1 * msg->axes[0];
        float LS_Y = msg->axes[1];
        float RS_X = -1 * msg->axes[3];
        float RS_Y = msg->axes[4];

        // bool CROSS = msg->buttons[0];
        // bool CIRCLE = msg->buttons[1];
        // bool TRIANGLE = msg->buttons[2];
        // bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        // bool UP = msg->axes[7] == 1.0;
        // bool DOWN = msg->axes[7] == -1.0;

        float L1 = msg->buttons[9];
        float R1 = msg->buttons[10];

        // float L2 = (-1 * msg->axes[2] + 1) / 2;
        float R2 = (-1 * msg->axes[5] + 1) / 2;

        // bool SHARE = msg->buttons[8];
        // // bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];

        // // bool L3 = msg->buttons[11];
        // // bool R3 = msg->buttons[12];

        // 　↓必要なのか不明

        /* if (PS == 1) {
             while (1) {
                 std::fill(data.begin(), data.end(), 0);
                 udp_.send(data);
                 std::cout << "！緊急停止中！" << std::endl;
               std::this_thread::sleep_for(std::chrono::milliseconds(100));
               return;
             }
         };*/
        // ９０〜９６まで必要なければコメントアウト
        float rad = atan2(LS_Y, LS_X);
        if (R2 >= 0.3) {
            v1 = sin(rad - 3 * M_PI / 4) * R2;
            v2 = sin(rad - 5 * M_PI / 4) * R2;
            v3 = sin(rad - 7 * M_PI / 4) * R2;
            v4 = sin(rad - 9 * M_PI / 4) * R2;
        }

        else if (RS_X >= deadzone || R1 == 1) {
            v1 = -1.0 * sp_yaw;
            v2 = -1.0 * sp_yaw;
            v3 = -1.0 * sp_yaw;
            v4 = -1.0 * sp_yaw;
        }

        else if (RS_X <= -1 * deadzone || L1 == 1) {
            v1 = 1.0 * sp_yaw;
            v2 = 1.0 * sp_yaw;
            v3 = 1.0 * sp_yaw;
            v4 = 1.0 * sp_yaw;
        }

        else if (
            (fabsf(LS_X) <= deadzone) && (fabsf(LS_Y) <= deadzone) && (fabsf(RS_X) <= deadzone) && (fabsf(RS_Y) <= deadzone) && (R1 == 0) && (L1 == 0)) {
            v1 = 0.0;
            v2 = 0.0;
            v3 = 0.0;
            v4 = 0.0;
        }

        // printf("\t\n%d,%d,%d,%d\n",v1, v2, v3, v4);
        data[1] = int(v1 * duty_max);
        data[2] = int(v2 * duty_max);
        data[3] = int(v3 * duty_max);
        data[4] = int(v4 * duty_max);
        udp_.send(data);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    UDP udp_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto ps4_listener = std::make_shared<PS4_Listener>(udp_ip, udp_port);
    exec.add_node(ps4_listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
