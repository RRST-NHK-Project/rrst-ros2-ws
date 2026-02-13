/*
RRST-NHK-Project 2026
PS4コントローラーの入力を取得するサンプルプログラム
esp32マイコンにアクチュエータ指令を送るサンプルプログラム

ボタン操作について
 上矢印：前輪上げ下げ
 下矢印：後輪上げ下げ
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

// 以下マイコンに合わせて設定
#define TARGET_DEVICE_ID 6 // 宛先マイコンのID //1,2,3 r1で使用中
#define TX16NUM 24         // 送信データ数
#define RX16NUM 17         // 受信データ数

#define MC_PRINTF 0 // マイコン側のprintfを無効化・有効化(0 or 1)

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

#define MAX_DEG 300

// constexpr size_t TX16NUM = 24;

// 定数・変数
float duty_max = 100;
float for_speed = 50;
float back_speed = 50;
float sp_yaw = 0.5;
float deadzone = 0.3; // adjust DS4 deadzone

float x = 0;
float y = 0;
float maxdeg1 = 70;
float mindeg1 = 0;
float maxdeg2 = 60;
float mindeg2 = 0;

float v1, v2, v3, v4; // 各メカナムホイールの速度指令値
// v1:第一象限, v2:第二象限, v3:第三象限, v4:第四象限

class Action
{
public:
    static void for_up(std::vector<int16_t> &data)
    {
        data[1] = 1;
    }
    static void for_down(std::vector<int16_t> &data)
    {
        data[1] = 0;
    }
    static void back_up(std::vector<int16_t> &data)
    {
        data[2] = 1;
    }
    static void back_down(std::vector<int16_t> &data)
    {
        data[2] = 0;
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
        x = msg->data[1];
        y = msg->data[2];
        // v4 = msg->data[3];
    }
};

class DriverInterface : public rclcpp::Node
{
public:
    DriverInterface(uint8_t device_id)
        : Node("driver_interface_" + std::to_string(device_id)),
          device_id_(device_id)
    {

        // 配列を0で初期化
        data_.assign(TX16NUM, 0);

        // joyノードのSubscribe
        // joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        //     "joy", 10,
        //     std::bind(&DriverInterface::IK_cal, this, std::placeholders::_1));

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
    // void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    // {

    //     // コントローラーの入力を取得、使わない入力はコメントアウト推奨
    //     // float LS_X = -1 * msg->axes[0];
    //     // float LS_Y = msg->axes[1];
    //     // float RS_X = -1 * msg->axes[3];
    //     // float RS_Y = msg->axes[4];

    //     // bool CROSS = msg->buttons[0];
    //     // bool CIRCLE = msg->buttons[1];
    //     // bool TRIANGLE = msg->buttons[2];
    //     // bool SQUARE = msg->buttons[3];

    //     // bool LEFT = msg->axes[6] == 1.0;
    //     // bool RIGHT = msg->axes[6] == -1.0;
    //     // bool UP = msg->axes[7] == 1.0;
    //     // bool DOWN = msg->axes[7] == -1.0;

    //     // bool L1 = msg->buttons[4];
    //     // bool R1 = msg->buttons[5];

    //     // float L2_DIGITAL = (-1 * msg->axes[2] + 1) / 2;
    //     // float R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2;

    //     // bool L2 = msg->buttons[6];
    //     // bool R2 = msg->buttons[7];

    //     // bool SHARE = msg->buttons[8];
    //     // bool OPTION = msg->buttons[9];
    //     // bool PS = msg->buttons[10];

    //     // bool L3 = msg->buttons[11];
    //     // bool R3 = msg->buttons[12];
    // }

    void publisher_timer_callback()
    {

        IK_cal();
        std_msgs::msg::Int16MultiArray msg;

        msg.data = data_;

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

    void
    sensor_callback(
        const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {
        // 最低限：サイズチェック
        if (msg->data.size() < RX16NUM)
        {
            RCLCPP_WARN(this->get_logger(),
                        "serial_rx_%d: data too short (%zu)",
                        device_id_, msg->data.size());
            return;
        }

        // int16_t ENC1 = msg->data[1];
        // int16_t ENC2 = msg->data[2];
        //  int16_t ENC3 = msg->data[3];
        //  int16_t ENC4 = msg->data[4];
        //  int16_t ENC5 = msg->data[5];
        //  int16_t ENC6 = msg->data[6];
        //  int16_t ENC7 = msg->data[7];
        //  int16_t ENC8 = msg->data[8];

        // int16_t SW1 = msg->data[9];
        // int16_t SW2 = msg->data[10];
        // int16_t SW3 = msg->data[11];
        // int16_t SW4 = msg->data[12];
        // int16_t SW5 = msg->data[13];
        // int16_t SW6 = msg->data[14];
        // int16_t SW7 = msg->data[15];
        // int16_t SW8 = msg->data[16];

        // 以降、受信データを使った処理を記述

        // 受信データ処理ここまで
    }
    void IK_cal() //(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        float th1 = 0;
        float th2 = 0;
        float l1 = 600;
        float l2 = 400;

        float r = sqrt(x * x + y * y);

        if (r > l1 + l2 || r < fabs(l1 - l2))
        {
            return; // 到達不能
        }
        th2 = acos(((x * x) + (y * y) - (l2 * l2) - (l1 * l1)) / (2 * l1 * l2));

        th1 = acos(((x * x) + (y * y) - (l2 * l2) + (l1 * l1)) / (2 * l1 * sqrt((x * x) * (y * y))));

        th1 = th1 * 180.0f / M_PI;
        th2 = th2 * 180.0f / M_PI;

        data_[1] = x;
        data_[2] = y;

        data_[5] = th1;
        data_[6] = th2;

        // 第一リンク0~70
        // 第二リンク60~0
        th2 = -th2 + 60;
        if (th1 > maxdeg1)
        {
            th1 = maxdeg1;
        }
        else if (th1 < mindeg1)
        {
            th1 = mindeg1;
        }
        else if (th2 > maxdeg2)
        {
            th2 = maxdeg2;
        }
        else if (th1 < mindeg1)
        {
            th2 = mindeg2;
        }

        data_[9] = th1;
        data_[10] = th2;
    }

    uint8_t device_id_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<int16_t> data_;
    std::mutex data_mutex_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;

    auto driver_interface = std::make_shared<DriverInterface>(TARGET_DEVICE_ID);
    auto omni_param = std::make_shared<Omni_para>();
    exec.add_node(driver_interface);
    exec.add_node(omni_param);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}