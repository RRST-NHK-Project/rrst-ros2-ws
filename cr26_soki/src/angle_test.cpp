/*
Serial_Bridgeノードのホスト側プログラム
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

#include <chrono>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <thread>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

// 自作 (common パッケージ)
#include "common/common.hpp"

// 以下マイコンに合わせて設定
#define TX_DEVICE_ID 3 // 送信先マイコンのID
#define RX_DEVICE_ID 3 // 受信先マイコンのID

#define TX16NUM 24 // 送信データ数
#define RX16NUM 24// 受信データ数

#define PUBLISH_RATE_MS 10 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

// スティックのデッドゾーン
#define DEADZONE_L 0.3
#define DEADZONE_R 0.3


class HardWareControl : public rclcpp::Node
{
public:
    HardWareControl(uint8_t tx_device_id, uint8_t rx_device_id)
        : Node("hardware_control_" + std::to_string(tx_device_id)),
          tx_device_id_(tx_device_id),
          rx_device_id_(rx_device_id)
    {

        // 配列を0で初期化
        pkt.data_.fill(0);
        /*
        マイコンに送信される配列"data_"
        debug: 機能未割り当て, MD: モータードライバー, TR: トランジスタ
        | data[n] | 詳細 | 範囲 |
        | ---- | ---- | ---- |
        | data[0] | debug | 0 or 1 |
        | data[1] | MD1 | -100 ~ 100 |
        | data[2] | MD2 | -100 ~ 100 |
        | data[3] | MD3 | -100 ~ 100 |
        | data[4] | MD4 | -100 ~ 100 |
        | data[5] | MD5 | -100 ~ 100 |
        | data[6] | MD6 | -100 ~ 100 |
        | data[7] | MD7 | -100 ~ 100 |
        | data[8] | MD8 | -100 ~ 100 |
        | data[9] | Servo1 | 0 ~ 270 |
        | data[10] | Servo2 | 0 ~ 270 |
        | data[11] | Servo3 | 0 ~ 270 |
        | data[12] | Servo4 | 0 ~ 270 |
        | data[13] | Servo5 | 0 ~ 270 |
        | data[14] | Servo6 | 0 ~ 270 |
        | data[15] | Servo7 | 0 ~ 270 |
        | data[16] | Servo8 | 0 ~ 270 |
        | data[17] | TR1 | 0 or 1|
        | data[18] | TR2 | 0 or 1|
        | data[19] | TR3 | 0 or 1|
        | data[20] | TR4 | 0 or 1|
        | data[21] | TR5 | 0 or 1|
        | data[22] | TR6 | 0 or 1|
        | data[23] | TR7 | 0 or 1|
        | data[24] | TR8 | 0 or 1|
        */

        // joyノードのSubscribe
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&HardWareControl::ps4_listener_callback, this, std::placeholders::_1));

        // seial_bridgeへpublish
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(tx_device_id_), 10);

        // timer_callbackを呼び出すタイマーを作成
        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&HardWareControl::publisher_timer_callback, this));

        sensor_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(rx_device_id_),
            10,
            std::bind(&HardWareControl::sensor_callback,
                      this,
                      std::placeholders::_1));

        pd_angle_.set_target(static_cast<float>(target_angle_deg_));
        pd_angle_.reset();

        RCLCPP_INFO(get_logger(),
                    "serial_tx_%d started.", tx_device_id_);
    }

private:
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {

        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        // float LS_X = -1 * msg->axes[0];
        // float LS_Y = msg->axes[1];
        // float RS_X = -1 * msg->axes[3];
        // float RS_Y = msg->axes[4];

        // bool CROSS = msg->buttons[0];
        // bool CIRCLE = msg->buttons[1];
        // bool TRIANGLE = msg->buttons[2];
        // bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        // bool L1 = msg->buttons[4];
        // bool R1 = msg->buttons[5];

        // float L2_DIGITAL = (-1 * msg->axes[2] + 1) / 2;
        // float R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2;

        //bool L2 = msg->buttons[6];
        //bool R2 = msg->buttons[7];

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        // static bool last_option = false;
        // static bool option_latch = false;

        // static bool last_share = false;
        // static bool share_latch = false;
               
        if (UP && !last_up_)
        {
            target_angle_deg_ += 90.0;
            pd_angle_.set_target(static_cast<float>(target_angle_deg_));
            pd_angle_.reset();
            // Initialize internal last_current_ to avoid a large derivative spike
            pd_angle_.update(static_cast<float>(enc1_total_angle_deg_), 1.0f);
            //RCLCPP_INFO(get_logger(), "Target angle updated: %.3f deg", target_angle_deg_);
        }

        if (DOWN && !last_down_)
        {
            target_angle_deg_ -= 90.0;
            pd_angle_.set_target(static_cast<float>(target_angle_deg_));
            pd_angle_.reset();
            // Initialize internal last_current_ to avoid a large derivative spike
            pd_angle_.update(static_cast<float>(enc1_total_angle_deg_), 1.0f);
            //RCLCPP_INFO(get_logger(), "Target angle updated: %.3f deg", target_angle_deg_);
        }
        last_up_ = UP;
        last_down_ = DOWN;

        // デバッグ用
        // RCLCPP_INFO(
        //     get_logger(),
        //     "data_[1-4]=[%d,%d,%d,%d], data_[9-12]=[%d,%d,%d,%d]",
        //     data_[1], data_[2], data_[3], data_[4],
        //     data_[9], data_[10], data_[11], data_[12]);

        // 配列操作ここまで
    }

    // publish
    void publisher_timer_callback()
    {
        std_msgs::msg::Int16MultiArray msg;

        msg.data = pkt.toVector();

        publisher_->publish(msg);
    }

    void
    sensor_callback(
        const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {

        int16_t enc1 = msg->data[1];
        // int16_t ENC2 = msg->data[2];
        // int16_t ENC3 = msg->data[3];
        // int16_t ENC4 = msg->data[4];
        // int16_t ENC5 = msg->data[5];
        // int16_t ENC6 = msg->data[6];
        // int16_t ENC7 = msg->data[7];
        // int16_t ENC8 = msg->data[8];

        // int16_t SW1 = msg->data[9];
        // int16_t SW2 = msg->data[10];
        // int16_t SW3 = msg->data[11];
        // int16_t SW4 = msg->data[12];
        // int16_t SW5 = msg->data[13];
        // int16_t SW6 = msg->data[14];
        // int16_t SW7 = msg->data[15];
        // int16_t SW8 = msg->data[16];

        // 以降、受信データを使った処理を記述
        //const int32_t HALF_ENCODER = 16384;   
        //const int64_t ENCODER_MAX = 32768;   //上限 

        const int32_t HALF_ENCODER = 4096;    // 1周(8192)の半分
        const int64_t ENCODER_MAX = 8192;     // エンコーダ1周あたりのカウント数

        if (!enc1_initialized_)
        {
            last_enc1_ = enc1;
            enc1_initialized_ = true;
            last_control_time_ = now();
        }
        else
        {
            const int32_t diff = static_cast<int32_t>(enc1) - last_enc1_;

            if (diff > HALF_ENCODER)
            {
                enc1_rotation_count_--;
            }
            else if (diff < -HALF_ENCODER)
            {
                enc1_rotation_count_++;
            }

            last_enc1_ = enc1;
        }

        enc1_total_encoder_ = static_cast<int64_t>(enc1_rotation_count_) * ENCODER_MAX + enc1;
        enc1_total_angle_deg_ = static_cast<double>(enc1_total_encoder_) * (360.0 / ENCODER_MAX);

        if (!target_initialized_)
        {
            target_angle_deg_ = enc1_total_angle_deg_;
            pd_angle_.set_target(static_cast<float>(target_angle_deg_));
            pd_angle_.reset();
            // prime last_current_ to avoid derivative spike on first update
            pd_angle_.update(static_cast<float>(enc1_total_angle_deg_), 1.0f);
            target_initialized_ = true;
        }

        const rclcpp::Time current_time = now();
        const double dt = (current_time - last_control_time_).seconds();
        last_control_time_ = current_time;

        const float command = pd_angle_.update(static_cast<float>(enc1_total_angle_deg_), static_cast<float>(dt));

        // smooth command to avoid sudden jumps (helps with derivative noise)
        const double smoothed_command = smoothing_alpha_ * prev_command_ + (1.0 - smoothing_alpha_) * static_cast<double>(command);
        prev_command_ = smoothed_command;

        // clamp to device-expected range (-100..100)
        const int md7_command = static_cast<int>(std::clamp(smoothed_command, -300.0, 300.0));

        pkt.setMD(MD7, md7_command);

        const double error_deg = target_angle_deg_ - enc1_total_angle_deg_;
        // RCLCPP_INFO(get_logger(), "ENC1: %.3f deg target: %.3f deg err: %.3f cmd: %.3f smoothed: %.3f md7: %d",
        //         enc1_total_angle_deg_, target_angle_deg_, error_deg, static_cast<double>(command), smoothed_command, md7_command);
        // 受信データ処理ここまで
    }

    uint8_t tx_device_id_;
    uint8_t rx_device_id_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool enc1_initialized_ = false;
    int32_t last_enc1_ = 0;
    int32_t enc1_rotation_count_ = 0;
    int64_t enc1_total_encoder_ = 0;
    double enc1_total_angle_deg_ = 0.0;
    bool last_up_ = false;
    bool last_down_ =false;
    bool target_initialized_ = false;
    double target_angle_deg_ = 0.0;
    rclcpp::Time last_control_time_;
    PDController pd_angle_{1.2f, 0.3f, 100.0f};

    // smoothing for controller output to prevent rapid jumps
    double prev_command_ = 0.0;
    const float smoothing_alpha_ = 0.7f; // higher -> more smoothing

    PacketController pkt;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet ! Welcome Day !";
    int result = std::system(figletout.c_str());
    if (result != 0)
    {
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
        std::cerr << "Please install 'figlet' with the following command:"
                  << std::endl;
        std::cerr << "sudo apt install figlet" << std::endl;
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
    }

    rclcpp::executors::MultiThreadedExecutor exec;

    auto hardware_control = std::make_shared<HardWareControl>(TX_DEVICE_ID, RX_DEVICE_ID);
    exec.add_node(hardware_control);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
