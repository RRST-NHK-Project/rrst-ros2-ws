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
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

// 自作
#include "include/PacketController.hpp"
PacketController pkt;
#include "include/PID.hpp"

#define TARGET_DEVICE_ID 6
#define PUBLISH_RATE_MS 50

class PIDMecanumController : public rclcpp::Node
{
public:
    PIDMecanumController()
        : Node("pid_mecanum_controller"),
          pid_x_(5.0, 0.0, 0.0, 1.0),
          pid_y_(5.0, 0.0, 0.0, 1.0),
          pid_yaw_(3.0, 0.0, 0.0, 1.0)
    {

        // odom subscriber
        odom_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "odom_xy_yaw", 10,
            std::bind(&PIDMecanumController::odom_callback, this, std::placeholders::_1));

        // マイコン送信
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(TARGET_DEVICE_ID), 10);

        // PS4入力
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&PIDMecanumController::ps4_listener_callback, this, std::placeholders::_1));

        // timer（制御周期固定）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&PIDMecanumController::publish_timer, this));

        // 初期target
        target_x_ = 0.0;
        target_y_ = 0.0;
        target_yaw_ = 0.0;
        pid_x_.set_target(target_x_);
        pid_y_.set_target(target_y_);
        pid_yaw_.set_target(target_yaw_);
    }

private:
    // ROS
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // PID
    PIDController pid_x_;
    PIDController pid_y_;
    PIDController pid_yaw_;

    // odom
    float X_ = 0.0;
    float Y_ = 0.0;
    float yaw_ = 0.0;

    // target
    float target_x_ = 0.0;
    float target_y_ = 0.0;
    float target_yaw_ = 0.0;

    // 制御出力
    float vx_ = 0.0;
    float vy_ = 0.0;
    float wz_ = 0.0;

    // mecanum
    float duty_max = 100;
    float v1 = 0, v2 = 0, v3 = 0, v4 = 0;

    // odom（状態更新のみ）
    void odom_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 3)
            return;

        X_ = msg->data[0];
        Y_ = msg->data[1];
        yaw_ = msg->data[2];
    }

    // PS4入力
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {

        float LS_X = -1 * msg->axes[0];
        float LS_Y = msg->axes[1];

        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;
        bool LEFT = msg->axes[6] == 1.0;
        bool RIGHT = msg->axes[6] == -1.0;

        static bool last_up = false;
        static bool last_down = false;
        static bool last_left = false;
        static bool last_right = false;

        // 目標変更（ここでPID更新＆リセット）
        if (UP && !last_up)
        {
            target_x_ = 0.3;
            target_y_ = 0.3;

            pid_x_.set_target(target_x_);
            pid_y_.set_target(target_y_);

            pid_x_.reset();
            pid_y_.reset();
        }

        if (DOWN && !last_down)
        {
            target_x_ = 0.0;
            target_y_ = 0.0;

            pid_x_.set_target(target_x_);
            pid_y_.set_target(target_y_);

            pid_x_.reset();
            pid_y_.reset();
        }

        if (LEFT && !last_left)
        {
            target_yaw_ += M_PI / 2; // 90度左回転

            pid_yaw_.set_target(target_yaw_);
            pid_yaw_.reset();
        }

        if (RIGHT && !last_right)
        {
            target_yaw_ -= M_PI / 2; // 90度右回転

            pid_yaw_.set_target(target_yaw_);
            pid_yaw_.reset();
        }

        last_up = UP;
        last_down = DOWN;
        last_left = LEFT;
        last_right = RIGHT;

        // float rad = atan2(LS_Y, LS_X);
    }
    // 制御ループ
    void publish_timer()
    {
        const float dt = PUBLISH_RATE_MS / 1000.0f;

        // PID計算
        vx_ = pid_x_.update(X_, dt);
        vy_ = -pid_y_.update(Y_, dt); // Y軸反転（座標系による）
        wz_ = -pid_yaw_.update(yaw_, dt);

        float cos_yaw = cos(-yaw_);
        float sin_yaw = sin(-yaw_);

        // オドメトリのデータをロボット座標系に変換
        float vx_robot = cos_yaw * vx_ + sin_yaw * vy_;
        float vy_robot = -sin_yaw * vx_ + cos_yaw * vy_;

        v1 = vy_robot + vx_robot + wz_;
        v3 = vy_robot - vx_robot - wz_;
        v4 = vy_robot - vx_robot + wz_;
        v2 = vy_robot + vx_robot - wz_;
        // メカナム逆運動学
        // v1 = vy_ + vx_ + wz_;
        // v3 = vy_ - vx_ - wz_;
        // v4 = vy_ - vx_ + wz_;
        // v2 = vy_ + vx_ - wz_;

        // 右側モータ反転
        v3 *= -1;
        v2 *= -1;

        // 正規化
        float max_v = std::max({fabs(v1), fabs(v2), fabs(v3), fabs(v4)});
        if (max_v < 1.0)
            max_v = 1.0;

        v1 /= max_v;
        v2 /= max_v;
        v3 /= max_v;
        v4 /= max_v;

        // 送信
        pkt.setMD(MD5, static_cast<int16_t>(v1 * duty_max));
        pkt.setMD(MD6, static_cast<int16_t>(v2 * duty_max));
        pkt.setMD(MD7, static_cast<int16_t>(v3 * duty_max));
        pkt.setMD(MD8, static_cast<int16_t>(v4 * duty_max));

        std_msgs::msg::Int16MultiArray msg;
        msg.data = pkt.toVector();
        publisher_->publish(msg);

        // デバッグ
        RCLCPP_INFO(this->get_logger(),
                    "X: %.2f Y: %.2f Yaw: %.2f | vx %.2f vy %.2f wz %.2f",
                    X_, Y_, yaw_, vx_, vy_, wz_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PIDMecanumController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}