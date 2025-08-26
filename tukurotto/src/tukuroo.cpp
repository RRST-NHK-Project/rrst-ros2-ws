/*
RRST-NHK-Project 2025
UDP通信を行うサンプルプログラム
動作確認まだ！注意！！
*/

/*
RRST-NHK-Project 2025
PS4コントローラーの入力を取得するサンプルプログラム
*/

// 標準
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

// 自作クラス
#include "include/IP.hpp"
#include "include/UDP.hpp"

#define MC_PRINTF 0 // マイコン側のprintfを無効化・有効化(0 or 1)

// L2,R2のデッドゾーン(0~1のうち0.3以上押されないと実行されないために？)
#define DEADZONE_L2 0.3
#define DEADZONE_R2 0.3

//CIRCLE,TRIANGLEが押された回数で変える
int CIRCLE_count = 0; // CIRCLEで巻き上げ、下ろすのをやりたい
int TRIANGLE_count = 0; // TRIANGLEでアームを伸び縮みさせたい
bool
bool

// 速度
int wheelspeed = ??;

// サーボ
int servo_angle = ??;

// サーボの組み付け時のズレを補正(度数法)
int SERVO1_CAL = ??;
int SERVO2_CAL = ??;
int SERVO3_CAL = ??;
int SERVO4_CAL = ??;
int SERVO5_CAL = ??;
int SERVO6_CAL = ??;
int SERVO7_CAL = ??;
int SERVO8_CAL = ??;


std::vector<int16_t> data(22, 0); // マイコンに送信される配列"data" std~は型　int16は16bit（通信量）
/*
マイコンに送信される配列"data"
debug: マイコンのprintfを有効化,  SN: ソレノイド,　MD:　モタドラ

| data[n] | 詳細 | 範囲 |
| ---- | ---- | ---- |
| data[0] | debug | 0 or 1 |
| data[1] | MD | -100 ~ 100 |
| data[2] | MD | -100 ~ 100 |
| data[3] | MD | -100 ~ 100 |
| data[4] | MD | -100 ~ 100 |
| data[5] | MD | -100 ~ 100 |
| data[6] | MD | -100 ~ 100 |
| data[7] | servo1 | 0 ~ 270 |
| data[8] | servo2 | 0 ~ 270 |
| data[9] | servo3 | 0 ~ 270 |
| data[10] | servo4 | 0 ~ 270 |
| data[11] | servo5 | 0 ~ 270 |
| data[12] | servo6 | 0 ~ 270 |
| data[13] | servo7 | 0 ~ 270 |
| data[14] | servo8 | 0 ~ 270 |
| data[15] | SN | 0 or 1 |
| data[16] | SN | 0 or 1 |
| data[17] | SN | 0 or 1 |
| data[18] | SN | 0 or 1 |
| data[19] | SN | 0 or 1 |
| data[20] | SN | 0 or 1 |
| data[21] | SN | 0 or 1 |
| data[22] | SN | 0 or 1 |
*/

class PS4_Listener : public rclcpp::Node {
public:
    PS4_Listener(const std::string &ip, int port)
        : Node("ps4_listener"), udp_(ip, port) {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "PS4 Listener initialized");
    }

private:
    // コントローラーの入力を取得、使わない入力はコメントアウト推奨
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

        data[0] = MC_PRINTF; // マイコン側のprintfを無効化・有効化(0 or 1)

        float L2 = (-1 * msg->axes[2] + 1) / 2;
        float R2 = (-1 * msg->axes[5] + 1) / 2;

        data[3] = 50 * R2; // 3番の最大を５０に設定
        udp_.send(data);   // データ送信
        
        bool CROSS = msg->buttons[0];
        bool CIRCLE = msg->buttons[1];
        bool TRIANGLE = msg->buttons[2];
        bool SQUARE = msg->buttons[3];
        bool L1 = msg->buttons[4];
        bool R1 = msg->buttons[5];
        bool L2 = msg->buttons[6];
        bool R2 = msg->buttons[7];
        bool SHARE = msg->buttons[8];
        bool OPTIONS = msg->buttons[9];
        bool PS = msg->buttons[10];
        bool LEFT = msg->axes[6] == 1.0;
        bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;
        
        if (CROSS) {
            std::cout << "CROSS" << std::endl;
        }
        if (CIRCLE) {
            std::cout << "CIRCLE" << std::endl;
        }
        if (TRINAGLE) {
            std::cout << "TRIANGLE" << std::endl;
        }
        if (SQUARE) {
            std::cout << "SQUARE" << std::endl;
        }
        if (L1) {
            std::cout << "L1" << std::endl;
        }
        if (R1) {
            std::cout << "R1" << std::endl;
        }
        if (SHARE) {
            std::cout << "SHARE" << std::endl;
        }
        if (OPTIONS) {
            std::cout << "OPTIONS" << std::endl;
        }
        if (PS) {
            std::cout << "PS" << std::endl;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    UDP udp_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto ps4_listener = std::make_shared<PS4_Listener>(IP_TEST, PORT_TEST);
    exec.add_node(ps4_listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}