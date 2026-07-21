/*
ros2can (MODE_ROBOMAS) 経由でDJIロボマスの角度制御を行うホスト側テストプログラム
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
// ros2can (xiao-esp32-s3_can2io, MODE_ROBOMAS) の DEVICE_ID / CAN_ID (指令送信先)
#define ROBOMAS_DEVICE_ID 201

// 制御対象のロボマスのインデックス (0-3: モータ1-4)
#define ROBOMAS_MOTOR_INDEX 0

// 位置フィードバック用AMTエンコーダを繋いだ別マイコンのDEVICE_ID (robomasとは別体)
#define ENCODER_DEVICE_ID 101

#define ARRAY_SIZE 24 // シリアルフレームのスロット数 (frame_data.hppのTx16NUM/Rx16NUMと同じ)

#define PUBLISH_RATE_MS 10 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

// スティックのデッドゾーン
#define DEADZONE_L 0.3
#define DEADZONE_R 0.3

// 目標rpmの安全クランプ (念のための二重の飽和、PD側のmax_outが本来の制限)
#define ROBOMAS_MAX_TARGET_RPM 300.0


class HardWareControl : public rclcpp::Node
{
public:
    HardWareControl(uint8_t robomas_device_id, uint8_t encoder_device_id)
        : Node("hardware_control_" + std::to_string(robomas_device_id)),
          robomas_device_id_(robomas_device_id),
          encoder_device_id_(encoder_device_id)
    {
        /*
        ros2can (MODE_ROBOMAS) への指令フレーム (robomas.cpp 参照、24スロット):

        送信 (PC -> 本機, serial_tx_[ROBOMAS_DEVICE_ID]、本機側Rx_16Data):
          0-3: target_rpm (モータ1-4、生のrpm値、スケール無し)
          4-23: 未使用

        位置フィードバックはrobomas自身の帰還ではなく、出力軸に直付けされた
        別体のAMTエンコーダ (serial_rx_[ENCODER_DEVICE_ID]) のdata[8]を使用する。
        このエンコーダは1周8192カウントだが、4周(32768カウント)で0にリセットされる
        多回転タイプのため、ラップ検出の周期は32768で行う (度への換算は1周=8192のまま)。
        */

        // joyノードのSubscribe
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&HardWareControl::ps4_listener_callback, this, std::placeholders::_1));

        // ros2can (robomas) へpublish (target_rpm指令)
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(robomas_device_id_), 10);

        // timer_callbackを呼び出すタイマーを作成
        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&HardWareControl::publisher_timer_callback, this));

        // AMTエンコーダ(robomasとは別体のマイコン)からのSubscribe
        sensor_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(encoder_device_id_),
            10,
            std::bind(&HardWareControl::sensor_callback,
                      this,
                      std::placeholders::_1));

        pd_angle_.set_target(static_cast<float>(target_angle_deg_));
        pd_angle_.reset();

        RCLCPP_INFO(get_logger(),
                    "serial_tx_%d started (encoder: serial_rx_%d).",
                    robomas_device_id_, encoder_device_id_);
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

        RCLCPP_INFO(
            get_logger(),
            "target_angle= %.1f, angle= %.1f, target_rpm= %d",
            target_angle_deg_, enc1_total_angle_deg_, target_rpm_command_
        );
        // 配列操作ここまで
    }

    // publish
    void publisher_timer_callback()
    {
        std_msgs::msg::Int16MultiArray msg;

        msg.data.assign(ARRAY_SIZE, 0);
        msg.data[ROBOMAS_MOTOR_INDEX] = target_rpm_command_;

        publisher_->publish(msg);
    }

    void
    sensor_callback(
        const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {
        // AMTエンコーダの生値 (1周8192カウント、4周=32768カウントで0にリセット)
        const int16_t enc1 = msg->data[8];

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
        // dtの下限をクランプ (エンコーダマイコンからのメッセージ間隔が数msまで詰まることがあり、
        // 微分項が (current-last)/dt で跳ね上がって振動の原因になるため)
        const double dt = std::max((current_time - last_control_time_).seconds(), 0.005);
        last_control_time_ = current_time;

        // 角度PDの出力をそのまま目標rpmとしてrobomasの速度ループへ渡す
        const float command = pd_angle_.update(static_cast<float>(enc1_total_angle_deg_), static_cast<float>(dt));

        // smooth command to avoid sudden jumps (helps with derivative noise)
        const double smoothed_command = smoothing_alpha_ * prev_command_ + (1.0 - smoothing_alpha_) * static_cast<double>(command);
        prev_command_ = smoothed_command;

        target_rpm_command_ = static_cast<int16_t>(
            std::clamp(smoothed_command, -ROBOMAS_MAX_TARGET_RPM, ROBOMAS_MAX_TARGET_RPM));

        // const double error_deg = target_angle_deg_ - angle_deg;
        // RCLCPP_INFO(get_logger(), "angle: %.3f deg target: %.3f deg err: %.3f cmd: %.3f smoothed: %.3f target_rpm: %d",
        //         angle_deg, target_angle_deg_, error_deg, static_cast<double>(command), smoothed_command, target_rpm_command_);
        // 受信データ処理ここまで
    }

    uint8_t robomas_device_id_;
    uint8_t encoder_device_id_;

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
    PDController pd_angle_{0.25f, 0.05f, 100.0f};

    // smoothing for controller output to prevent rapid jumps
    double prev_command_ = 0.0;
    const float smoothing_alpha_ = 0.7f; // higher -> more smoothing

    int16_t target_rpm_command_ = 0;
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

    auto hardware_control = std::make_shared<HardWareControl>(ROBOMAS_DEVICE_ID, ENCODER_DEVICE_ID);
    exec.add_node(hardware_control);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
