/*
RRST-NHK-Project 2025
PS4コントローラーの入力を取得するサンプルプログラム
esp32マイコンにアクチュエータ指令を送るサンプルプログラム
*/

// 標準
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include "std_msgs/msg/int32_multi_array.hpp"
// #include "actuator_msg/msg/actuator_msg.hpp"
#include <vector>

#define MC_PRINTF 0 // マイコン側のprintfを無効化・有効化(0 or 1)

// 定数・変数
float duty_max = 500;
float sp_yaw = 0.5;

float deadzone = 0.3; // adjust DS4 deadzone

float v1, v2, v3, v4; // 各オムニホイールの速度指令値

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

std::vector<int16_t> data(18, 0); // マイコンに送信される配列"data"

// 各機構シーケンスを格納するクラス
class Action
{
public:
    static void tokei()
    {
        //    data[3] -= turn_speed*deg;
    }
};

class PS4_Listener : public rclcpp::Node
{
public:
    PS4_Listener()
        : Node("ps4_listener")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this,
                      std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("to_esp32_1", 10);

        RCLCPP_INFO(this->get_logger(),
                    "PS4 Listener initialized");
    }

private:
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {

        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
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

        // static bool last_share = false; // 前回の状態を保持する static 変数
        // static bool share_latch = false;

        // static bool last_R1 =false;
        //  static bool last_L1 =false;
        // static bool last_triangle = false;

        data[0] = MC_PRINTF; // マイコン側のprintfを無効化・有効化(0 or 1)

        float rad = atan2(LS_Y, LS_X);
        if (R2_DIGITAL >= 0.3)
        {
            v1 = sin(rad - 3 * M_PI / 4) * R2_DIGITAL;
            v2 = sin(rad - 5 * M_PI / 4) * R2_DIGITAL;
            v3 = sin(rad - 7 * M_PI / 4) * R2_DIGITAL;
            v4 = sin(rad - 9 * M_PI / 4) * R2_DIGITAL;
        }

        else if (RS_X >= deadzone || R1 == 1)
        {
            v1 = -1.0 * sp_yaw;
            v2 = -1.0 * sp_yaw;
            v3 = -1.0 * sp_yaw;
            v4 = -1.0 * sp_yaw;
        }

        else if (RS_X <= -1 * deadzone || L1 == 1)
        {
            v1 = 1.0 * sp_yaw;
            v2 = 1.0 * sp_yaw;
            v3 = 1.0 * sp_yaw;
            v4 = 1.0 * sp_yaw;
        }

        else if (
            (fabsf(LS_X) <= deadzone) && (fabsf(LS_Y) <= deadzone) && (fabsf(RS_X) <= deadzone) && (fabsf(RS_Y) <= deadzone) && (R1 == 0) && (L1 == 0))
        {
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

        // デバッグ用（for文でcoutするとカクつく）
        std::cout << data[0] << ", " << data[1] << ", " << data[2] << ", " << data[3] << ", "<<std::endl;//", ";
        // std::cout << data[4] << ", " << data[5] << ", " << data[6] << ", " << data[7] << ", ";
        // std::cout << data[8] << ", " << data[9] << ", " << data[10] << ", " << data[11] << ", ";
        // std::cout << data[12] << ", " << data[13] << ", " << data[14] << ", " << data[15] << ", ";
        // std::cout << data[16] << ", " << data[17] << ", " << data[18] << std::endl;

        publish_data();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    void publish_data()
    {
        auto msg = std_msgs::msg::Int32MultiArray();
        msg.data.reserve(data.size());
        for (auto &v : data)
        {
            msg.data.push_back(static_cast<int32_t>(v));
        }
        publisher_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto ps4_listener = std::make_shared<PS4_Listener>();
    exec.add_node(ps4_listener);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}