/*
R2 メカナム Pure Pursuit ノード (Yaw固定)
Copyright (c) 2025 RRST-NHK-Project
*/

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class MecanumPurePursuit : public rclcpp::Node {
public:
    MecanumPurePursuit()
        : Node("r2_mecanum_pure_pursuit") {
        // odom subscriber (Float32MultiArray: X,Y,Yaw)
        odom_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "odom", 10,
            std::bind(&MecanumPurePursuit::odom_callback, this, std::placeholders::_1));

        // cmd_vel publisher
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // パラメータ
        this->declare_parameter<double>("look_ahead_distance", 0.4);
        this->declare_parameter<double>("target_speed", 0.5);

        this->get_parameter("look_ahead_distance", L_d_);
        this->get_parameter("target_speed", target_speed_);

        // サンプル経路 (x,y)
        waypoints_ = {
            {0.0, 0.0}, {-0.3, 0.3}, {0.3, 0.3}, {0.5, 0.5}, {0.0, 0.0}};
        current_goal_idx_ = 0;
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    std::vector<std::pair<double, double>> waypoints_;
    size_t current_goal_idx_;

    double X_ = 0.0;
    double Y_ = 0.0;
    double yaw_ = 0.0;

    double L_d_ = 0.4;          // look-ahead距離
    double target_speed_ = 0.5; // m/s

    void odom_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 3) {
            RCLCPP_WARN(get_logger(), "odom data too short");
            return;
        }

        // odom 取得
        X_ = msg->data[0];
        Y_ = msg->data[1];
        yaw_ = msg->data[2]; // rad

        // 次のターゲット点を選択
        auto goal = waypoints_[current_goal_idx_];
        double dx = goal.first - X_;
        double dy = goal.second - Y_;
        double dist = std::sqrt(dx * dx + dy * dy);

        // ゴールに近ければ次の waypoint へ
        if (dist < 0.1 && current_goal_idx_ + 1 < waypoints_.size()) {
            current_goal_idx_++;
            goal = waypoints_[current_goal_idx_];
            dx = goal.first - X_;
            dy = goal.second - Y_;
            dist = std::sqrt(dx * dx + dy * dy);
        }

        // ロボット座標系に変換（yaw固定）
        double cos_yaw = std::cos(-yaw_);
        double sin_yaw = std::sin(-yaw_);
        double vx = cos_yaw * dx - sin_yaw * dy;
        double vy = sin_yaw * dx + cos_yaw * dy;

        // ベクトルを目標速度にスケーリング
        double length = std::sqrt(vx * vx + vy * vy);
        if (length > 0.001) {
            vx = (vx / length) * target_speed_;
            vy = (vy / length) * target_speed_;
        } else {
            vx = 0.0;
            vy = 0.0;
        }

        // cmd_vel 作成
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = vx;
        cmd.linear.y = vy;
        cmd.angular.z = 0.0; // Yaw固定

        cmd_pub_->publish(cmd);

        // デバッグ
        RCLCPP_INFO(get_logger(),
                    "X: %.3f Y: %.3f | goal: %.2f %.2f | vx: %.2f vy: %.2f",
                    X_, Y_, goal.first, goal.second, vx, vy);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MecanumPurePursuit>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}