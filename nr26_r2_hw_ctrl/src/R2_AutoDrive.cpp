/*
R2 PID メカナム制御ノード
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

// 標準
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

// 自作
#include "include/PacketController.hpp"
PacketController pkt;
#include "include/PID.hpp"

#define TARGET_DEVICE_ID 6
#define PUBLISH_RATE_MS 50

class PIDMecanumController : public rclcpp::Node {
public:
    PIDMecanumController()
        : Node("pid_mecanum_controller"),
          pid_x_(5.0, 0.0, 0.0, 1.0),
          pid_y_(5.0, 0.0, 0.0, 1.0),
          pid_yaw_(3.0, 0.0, 0.0, 1.0) {

        // odom subscriber
        odom_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "odom_xy_yaw", 10,
            std::bind(&PIDMecanumController::odom_callback, this, std::placeholders::_1));

        // マイコン送信
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(TARGET_DEVICE_ID), 10);

        // GUI表示用モード配信
        mode_pub_ = this->create_publisher<std_msgs::msg::String>(
            "r2_drive_mode", rclcpp::QoS(1).transient_local().reliable());

        // PS4入力
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&PIDMecanumController::ps4_listener_callback, this, std::placeholders::_1));

        // r2_console からの目標位置/姿勢
        target_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "r2_autodrive_cmd", 10,
            std::bind(&PIDMecanumController::target_callback, this, std::placeholders::_1));

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
        publish_mode();
    }

private:
    // ROS
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
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
    bool auto_mode_ = false;
    bool has_target_cmd_ = false;

    // 制御出力
    float vx_ = 0.0;
    float vy_ = 0.0;
    float wz_ = 0.0;

    // mecanum
    float duty_max = 100;
    float sp_yaw = 0.5;
    float deadzone = 0.3;
    float v1 = 0, v2 = 0, v3 = 0, v4 = 0;

    void publish_mode() {
        std_msgs::msg::String mode_msg;
        mode_msg.data = auto_mode_ ? "AUTO" : "MANUAL";
        mode_pub_->publish(mode_msg);
    }

    // odom（状態更新のみ）
    void odom_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 3)
            return;

        X_ = msg->data[0];
        Y_ = msg->data[1];
        yaw_ = msg->data[2];
    }

    void target_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 3)
            return;

        target_x_ = msg->data[0];
        target_y_ = msg->data[1];
        target_yaw_ = msg->data[2];
        has_target_cmd_ = true;

        pid_x_.set_target(target_x_);
        pid_y_.set_target(target_y_);
        pid_yaw_.set_target(target_yaw_);
        pid_x_.reset();
        pid_y_.reset();
        pid_yaw_.reset();

        if (!auto_mode_) {
            auto_mode_ = true;
            publish_mode();
            RCLCPP_INFO(this->get_logger(), "Mode changed: AUTO (by target command)");
        }

        RCLCPP_INFO(this->get_logger(),
                    "Target updated x=%.3f y=%.3f yaw=%.3f [rad]",
                    target_x_, target_y_, target_yaw_);
    }

    // PS4入力
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() < 8 || msg->buttons.size() < 10)
            return;

        bool OPTION = msg->buttons[9];
        static bool last_option = false;

        if (OPTION && !last_option) {
            auto_mode_ = !auto_mode_;

            if (auto_mode_) {
                // 受信済み目標がない場合のみ現在位置ホールドから開始
                if (!has_target_cmd_) {
                    target_x_ = X_;
                    target_y_ = Y_;
                    target_yaw_ = yaw_;
                }
                pid_x_.set_target(target_x_);
                pid_y_.set_target(target_y_);
                pid_yaw_.set_target(target_yaw_);
                pid_x_.reset();
                pid_y_.reset();
                pid_yaw_.reset();
                publish_mode();
                RCLCPP_INFO(this->get_logger(), "Mode changed: AUTO");
            } else {
                // 手動モード移行時は一旦停止
                pkt.setMD(MD5, 0);
                pkt.setMD(MD6, 0);
                pkt.setMD(MD7, 0);
                pkt.setMD(MD8, 0);
                publish_mode();
                RCLCPP_INFO(this->get_logger(), "Mode changed: MANUAL");
            }
        }
        last_option = OPTION;

        if (auto_mode_) {
            return;
        }

        float LS_X = -msg->axes[0];
        float LS_Y = msg->axes[1];
        float RS_X = -msg->axes[3];
        float R2 = (-msg->axes[5] + 1) / 2;

        bool L1 = msg->buttons[4];
        bool R1 = msg->buttons[5];

        if (fabsf(LS_X) < deadzone)
            LS_X = 0;
        if (fabsf(LS_Y) < deadzone)
            LS_Y = 0;
        if (fabsf(RS_X) < deadzone)
            RS_X = 0;

        float vx = -LS_Y * R2;
        float vy = LS_X * R2;
        float wz = RS_X * sp_yaw;

        // SequenceCtrl と同じ手動メカナム計算
        v1 = vx + vy + wz;
        v3 = vx - vy - wz;
        v4 = vx - vy + wz;
        v2 = vx + vy - wz;

        v3 *= -1;
        v2 *= -1;

        if (R1) {
            v1 = sp_yaw;
            v2 = -sp_yaw;
            v3 = -sp_yaw;
            v4 = sp_yaw;
        }

        if (L1) {
            v1 = -sp_yaw;
            v2 = sp_yaw;
            v3 = sp_yaw;
            v4 = -sp_yaw;
        }

        float max_v = std::max(
            std::max(fabsf(v1), fabsf(v2)),
            std::max(fabsf(v3), fabsf(v4)));

        if (max_v < 1.0f)
            max_v = 1.0f;

        v1 /= max_v;
        v2 /= max_v;
        v3 /= max_v;
        v4 /= max_v;

        pkt.setMD(MD5, static_cast<int16_t>(v1 * duty_max));
        pkt.setMD(MD6, static_cast<int16_t>(v2 * duty_max));
        pkt.setMD(MD7, static_cast<int16_t>(v3 * duty_max));
        pkt.setMD(MD8, static_cast<int16_t>(v4 * duty_max));
    }
    // 制御ループ
    void publish_timer() {
        if (!auto_mode_) {
            std_msgs::msg::Int16MultiArray msg;
            msg.data = pkt.toVector();
            publisher_->publish(msg);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 50,
                                 "[MANUAL] X: %.2f Y: %.2f Yaw: %.2f | v1 %.2f v2 %.2f v3 %.2f v4 %.2f",
                                 X_, Y_, yaw_, v1, v2, v3, v4);
            return;
        }

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

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 50,
                             "[AUTO] X: %.2f Y: %.2f Yaw: %.2f | T: %.2f %.2f %.2f | vx %.2f vy %.2f wz %.2f",
                             X_, Y_, yaw_, target_x_, target_y_, target_yaw_, vx_, vy_, wz_);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PIDMecanumController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}