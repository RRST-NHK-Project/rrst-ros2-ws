/*
R2オドメトリ発行ノード（F横、L/R前進、R符号反転済み）
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

// 標準
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

// ROS
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

// 自作
#include "PacketController.hpp"
PacketController pkt;

// マイコン設定
#define TARGET_DEVICE_ID 6
#define TX16NUM 24
#define RX16NUM 17
#define PUBLISH_RATE_MS 20 // ms

using namespace std::chrono_literals;

class HardWareControl : public rclcpp::Node {
public:
    HardWareControl(uint8_t device_id)
        : Node("hardware_control_" + std::to_string(device_id)),
          device_id_(device_id) {

        sensor_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(device_id_),
            10,
            std::bind(&HardWareControl::sensor_callback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("odom_3", 10);

        odom_nav_pub_ =
            this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(get_logger(), "serial_rx_%d subscriber started.", device_id_);
    }

private:
    // オドメトリ設定
    static constexpr double ODOM_WHEEL_DIAMETER = 0.05;
    static constexpr double ODOM_WHEEL_RADIUS = ODOM_WHEEL_DIAMETER / 2.0;
    static constexpr double ODOM_WHEEL_CIRC = M_PI * ODOM_WHEEL_DIAMETER;
    static constexpr double ENCODER_RESOLUTION = 1024.0;
    static constexpr double ENC_TO_M = ODOM_WHEEL_CIRC / ENCODER_RESOLUTION;
    static constexpr double ODOM_LR_DISTANCE = 0.385;
    static constexpr double ODOM_F_OFFSET = 0.335;

    static constexpr double ODOM_X_SCALE = 1.0;
    static constexpr double ODOM_Y_SCALE = 1.0;
    static constexpr double ODOM_YAW_SCALE = 1.0;

    int16_t prev_f_ = 0;
    int16_t prev_l_ = 0;
    int16_t prev_r_ = 0;
    bool enc_init_ = false;

    double X = 0.0;
    double Y = 0.0;
    double yaw_ = 0.0;

    void sensor_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        if (msg->data.size() < RX16NUM) {
            RCLCPP_WARN(get_logger(),
                        "serial_rx_%d: data too short (%zu)",
                        device_id_, msg->data.size());
            return;
        }

        // オドメトリ（X,Y,YAWのみ）

        // エンコーダ取得
        int16_t enc_f = msg->data[1];  // F:横方向
        int16_t enc_l = msg->data[2];  // L:前進方向
        int16_t enc_r = -msg->data[3]; // R:前進方向、符号反転済み

        if (!enc_init_) {
            prev_f_ = enc_f;
            prev_l_ = enc_l;
            prev_r_ = enc_r;
            enc_init_ = true;
            return;
        }

        // 差分
        int16_t df = enc_f - prev_f_;
        int16_t dl = enc_l - prev_l_;
        int16_t dr = enc_r - prev_r_;

        prev_f_ = enc_f;
        prev_l_ = enc_l;
        prev_r_ = enc_r;

        // パルス → 距離
        double dF = df * ENC_TO_M; // 横スライド
        double dL = dl * ENC_TO_M; // 前進
        double dR = dr * ENC_TO_M; // 前進

        // 3輪オドメトリ
        // F:横方向、L/R:前進・回転
        double dx_r = dF;                             // 機体右方向（X軸）
        double dy_r = (dL + dR) / 2.0;                // 前方向（Y軸）
        double dtheta = (dR - dL) / ODOM_LR_DISTANCE; // 回転量

        // キャリブレーション
        dx_r *= ODOM_X_SCALE;
        dy_r *= ODOM_Y_SCALE;
        dtheta *= ODOM_YAW_SCALE;

        // ワールド座標変換
        double dx = dx_r * cos(yaw_) - dy_r * sin(yaw_);
        double dy = dx_r * sin(yaw_) + dy_r * cos(yaw_);

        // 位置更新
        X += dx;
        Y += dy;
        yaw_ += dtheta;
        yaw_ = atan2(sin(yaw_), cos(yaw_));

        // 速度計算
        float dt = PUBLISH_RATE_MS / 1000.0f;
        float vx = dx / dt;
        float vy = dy / dt;
        float wz = dtheta / dt;

        // Publish
        std_msgs::msg::Float32MultiArray odom_msg;
        odom_msg.data = {static_cast<float>(X),
                         static_cast<float>(Y),
                         static_cast<float>(yaw_),
                         vx, vy, wz};
        odom_pub_->publish(odom_msg);

        RCLCPP_INFO(get_logger(),
                    "X: %.3f  Y: %.3f  Yaw: %.2f deg",
                    X, Y, yaw_ * 180.0 / M_PI);

        // オドメトリ（完全な）
        // nav_msgs/Odometry publish

        auto now = this->get_clock()->now();

        nav_msgs::msg::Odometry odom;

        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = X;
        odom.pose.pose.position.y = Y;
        odom.pose.pose.position.z = 0.0;

        // quaternion
        double qz = sin(yaw_ * 0.5);
        double qw = cos(yaw_ * 0.5);

        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = qz;
        odom.pose.pose.orientation.w = qw;

        // velocity
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = wz;

        odom_nav_pub_->publish(odom);

        // TF broadcast

        geometry_msgs::msg::TransformStamped tf;

        tf.header.stamp = now;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";

        tf.transform.translation.x = X;
        tf.transform.translation.y = Y;
        tf.transform.translation.z = 0.0;

        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = qz;
        tf.transform.rotation.w = qw;

        tf_broadcaster_->sendTransform(tf);
    }

    uint8_t device_id_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr odom_pub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sensor_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_nav_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::string figletout = "figlet R2_Ext_Odom";
    int result = std::system(figletout.c_str());
    if (result != 0) {
        std::cerr << "Please install 'figlet' with: sudo apt install figlet\n";
    }

    rclcpp::executors::MultiThreadedExecutor exec;
    auto hardware_control = std::make_shared<HardWareControl>(TARGET_DEVICE_ID);
    exec.add_node(hardware_control);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}