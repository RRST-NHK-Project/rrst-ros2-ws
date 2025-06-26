/*
ビルド通すために書いてるだけ！！

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

class PS4_Listener : public rclcpp::Node {
public:
    PS4_Listener()
        : Node("ps4_listener") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "PS4 Listener initialized");
    }

private:
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

        bool CIRCLE = msg->buttons[1];
        if (CIRCLE) {
            std::cout << "CIRCLE" << std::endl;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto ps4_listener = std::make_shared<PS4_Listener>();
    exec.add_node(ps4_listener);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}