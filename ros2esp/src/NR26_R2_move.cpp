/*
RRST-NHK-Project 2026
PS4コントローラーの入力を取得するサンプルプログラム
esp32マイコンにアクチュエータ指令を送るサンプルプログラム

2025/11/20
初期化してもどこかが暴走する問題発生
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
#include "std_msgs/msg/int16_multi_array.hpp"

#include <mutex>

#define MC_PRINTF 0 // マイコン側のprintfを無効化・有効化(0 or 1)

constexpr size_t TX16NUM = 24;

// 定数・変数
float duty_max = 100;
float for_speed = 50;
float back_speed = 50;
float sp_yaw = 0.5;
float deadzone = 0.3; // adjust DS4 deadzone
float m1 = 3;
float m2 = 3;

float v1, v2, v3, v4; // 各メカナムホイールの速度指令値
// v1:第一象限, v2:第二象限, v3:第三象限, v4:第四象限

class Action
{
public:
    static void all_up(std::vector<int16_t> &data)
    {
        data[1] += m1;
        data[2] -= m2;
    }
    static void all_down(std::vector<int16_t> &data)
    {
        data[1] -= m1;
        data[2] += m2;
    }
};

class Omni_para : public rclcpp::Node
{
public:
    Omni_para() : Node("omni_tuner")
    {

        sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "omni_param",
            10,
            std::bind(&Omni_para::param_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "OMNI Node Started.");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
    std::vector<float> params = {0, 0, 0, 0}; // 受信したパラメータを保持

    void param_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {

        if (msg->data.size() < 4)
        {
            RCLCPP_WARN(this->get_logger(), "param message too short");
            return;
        }

        // 受信した params をコピー
        duty_max = msg->data[0];
        for_speed = msg->data[1];
        back_speed = msg->data[2];
        // v4 = msg->data[3];

        //  data[0] = params[0]; // v1
        //  data[1] = params[1]; // v2
        //  data[2] = params[2]; // v3
        //  data[3] = params[3]; // v4

        // // ここでモータ制御に送ったり何でもできる
        // std::cout << "Received params:" << std::endl;
        // std::cout << " v1=" << params[0]
        //           << " v2=" << params[1]
        //           << " v3=" << params[2]
        //           << " v4=" << params[3]
        //           << std::endl;
    }
};

class PS4_Listener : public rclcpp::Node
{
public:
    PS4_Listener(uint8_t device_id)
        : Node("ps4_listener"),
          device_id_(device_id)
    {
        data_.assign(TX16NUM, 0);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this,
                      std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(device_id_), 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100), // 20Hz
            std::bind(&PS4_Listener::timer_callback, this));

        RCLCPP_INFO(get_logger(),
                    "PS4 → serial_tx_%d started (timer publish)", device_id_);
    }

private:
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {

        std::lock_guard<std::mutex> lock(data_mutex_);
        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        float LS_X = -1 * msg->axes[0];
        float LS_Y = msg->axes[1];
        float RS_X = -1 * msg->axes[3];
        float RS_Y = msg->axes[4];

        // bool CROSS = msg->buttons[0];
        bool CIRCLE = msg->buttons[1];
        // bool TRIANGLE = msg->buttons[2];
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

        // static bool last_share = false; // 前回の状態を保持する static 変数
        // static bool share_latch = false;

        // static bool last_R1 =false;
        //  static bool last_L1 =false;
        // static bool last_triangle = false;

        float rad = atan2(LS_Y, LS_X);

        // float X = LS_X;
        // float Y = LS_Y;
        // float W = RS_X;

        if (R2_DIGITAL >= 0.3)
        {
            float vx = cos(rad) * R2_DIGITAL;
            float vy = sin(rad) * R2_DIGITAL;

            v1 = vy - vx; // 前右
            v2 = vy + vx; // 前左
            v3 = vy - vx; // 後左
            v4 = vy + vx; // 後右
        }
        else if (RS_X >= deadzone || R1 == 1)
        {
            v1 = sp_yaw;
            v2 = -sp_yaw;
            v3 = sp_yaw;
            v4 = -sp_yaw;
        }
        else if (RS_X <= -deadzone || L1 == 1)
        {
            v1 = -sp_yaw;
            v2 = sp_yaw;
            v3 = -sp_yaw;
            v4 = sp_yaw;
        }

        else if (
            (fabsf(LS_X) <= deadzone) && (fabsf(LS_Y) <= deadzone) && (fabsf(RS_X) <= deadzone) && (fabsf(RS_Y) <= deadzone) && (R1 == 0) && (L1 == 0))
        {
            v1 = 0.0;
            v2 = 0.0;
            v3 = 0.0;
            v4 = 0.0;
        }

        // 正規化(これで全方向安定した速度出せる)
        float max_v = std::max({fabsf(v1), fabsf(v2), fabsf(v3), fabsf(v4), 1.0f});

        v1 /= max_v;
        v2 /= max_v;
        v3 /= max_v;
        v4 /= max_v;

        if (UP)
        {
            Action::all_up(data_);
        }
        else if (DOWN)
        {
            Action::all_down(data_);
        }

        if (CIRCLE)
        {
            data_[1] = 0;
            data_[2] = 0;
        }

        // if (R2_DIGITAL >= 0.3)
        // {
        //     v1 = sin(rad - 3 * M_PI / 4) * R2_DIGITAL;
        //     v2 = sin(rad - 5 * M_PI / 4) * R2_DIGITAL;
        //     v3 = sin(rad - 7 * M_PI / 4) * R2_DIGITAL;
        //     v4 = sin(rad - 9 * M_PI / 4) * R2_DIGITAL;
        // }

        // else if (RS_X >= deadzone || R1 == 1)
        // {
        //     v1 = -1.0 * sp_yaw;
        //     v2 = -1.0 * sp_yaw;
        //     v3 = -1.0 * sp_yaw;
        //     v4 = -1.0 * sp_yaw;
        // }

        // else if (RS_X <= -1 * deadzone || L1 == 1)
        // {
        //     v1 = 1.0 * sp_yaw;
        //     v2 = 1.0 * sp_yaw;
        //     v3 = 1.0 * sp_yaw;
        //     v4 = 1.0 * sp_yaw;
        // }

        // else if (
        //     (fabsf(LS_X) <= deadzone) && (fabsf(LS_Y) <= deadzone) && (fabsf(RS_X) <= deadzone) && (fabsf(RS_Y) <= deadzone) && (R1 == 0) && (L1 == 0))
        // {
        //     v1 = 0.0;
        //     v2 = 0.0;
        //     v3 = 0.0;
        //     v4 = 0.0;
        // }

        // printf("\t\n%d,%d,%d,%d\n", v1, v2, v3, v4);
        // std::lock_guard<std::mutex> lock(data_mutex_);
        data_[7] = static_cast<int16_t>(v1 * duty_max);
        data_[8] = static_cast<int16_t>(v2 * duty_max);
        data_[9] = static_cast<int16_t>(v3 * duty_max);
        data_[10] = static_cast<int16_t>(v4 * duty_max);

        // std::cout << "\n"
        //           << data_[1] << "," << data_[2] << "," << data_[7] << "," << data_[8] << "," << data_[9] << "," << data_[10] << std::endl;

        // デバッグ用（for文でcoutするとカクつく）
        // std::cout << data[0] << ", " << data[1] << ", " << data[2] << ", " << data[3] << ", "<<std::endl;//", ";
        // std::cout << data[4] << ", " << data[5] << ", " << data[6] << ", " << data[7] << ", ";
        // std::cout << data[8] << ", " << data[9] << ", " << data[10] << ", " << data[11] << ", ";
        // std::cout << data[12] << ", " << data[13] << ", " << data[14] << ", " << data[15] << ", ";
        // std::cout << data[16] << ", " << data[17] << ", " << data[18] << std::endl;
    }

    /* ---------- 周期 publish ---------- */
    void timer_callback()
    {
        std_msgs::msg::Int16MultiArray msg;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            msg.data = data_;
        }
        publisher_->publish(msg);
        print_data();
    }

    void print_data()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        std::cout << "TX DATA: [";
        for (size_t i = 0; i < data_.size(); ++i)
        {
            std::cout << data_[i];
            if (i + 1 < data_.size())
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
    uint8_t device_id_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<int16_t> data_;
    std::mutex data_mutex_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;

    uint8_t device_id = 2; // serial_bridge_node と一致
    auto ps4_listener = std::make_shared<PS4_Listener>(device_id);
    auto omni_param = std::make_shared<Omni_para>();
    exec.add_node(ps4_listener);
    exec.add_node(omni_param);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}