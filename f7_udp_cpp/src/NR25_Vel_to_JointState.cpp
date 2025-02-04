/*
RRST NHK2025
Velocity to JointState
*/

// 標準
#include <chrono>
#include <thread>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>

class ENC_Listener : public rclcpp::Node {
public:
    ENC_Listener()
        : Node("nhk25_vel2js") {
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "enc", 10,
            std::bind(&ENC_Listener::enc_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "NHK2025 MR Vel to JointState");
    }

private:
    void enc_listener_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    float v1 = msg->data[0]; 
    float v2 = msg->data[1]; 
    float v3 = msg->data[2]; 
    float v4 = msg->data[3]; 
    std::cout << v1 << " " << v2 << " " << v3 << " " << v4 <<std::endl;
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto enc_listener = std::make_shared<ENC_Listener>();
    exec.add_node(enc_listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
