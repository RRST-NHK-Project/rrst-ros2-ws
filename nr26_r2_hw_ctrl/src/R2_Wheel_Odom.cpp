#include <chrono>
#include <cmath>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

using namespace std::chrono_literals;

// ===== 設定 =====
#define TARGET_DEVICE_ID 6
#define RX16NUM 17

constexpr float WHEEL_RADIUS = 0.05f;

// ==================

class WheelOdomNode : public rclcpp::Node {
public:
    WheelOdomNode() : Node("wheel_odom_node") {
        rx_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(TARGET_DEVICE_ID),
            10,
            std::bind(&WheelOdomNode::rx_callback, this, std::placeholders::_1));

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            "/wheel_odom",
            10);

        RCLCPP_INFO(get_logger(), "Wheel odom started");
    }

private:
    float X = 0.0f;
    float Y = 0.0f;

    rclcpp::Time last_time;
    bool odom_init = false;

    void rx_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        if (msg->data.size() < RX16NUM)
            return;

        int16_t vel[4] =
            {
                msg->data[11],
                msg->data[12],
                msg->data[13],
                msg->data[14]};

        auto current_time = this->now();

        if (!odom_init) {
            last_time = current_time;
            odom_init = true;
            return;
        }

        float dt = (current_time - last_time).seconds();
        last_time = current_time;

        if (dt <= 0.0f)
            return;

        float w[4];

        for (int i = 0; i < 4; i++) {
            w[i] = vel[i] * 2.0f * M_PI / 60.0f * WHEEL_RADIUS;
        }

        float vy = (w[0] - w[1] - w[2] + w[3]) / 4.0f;
        float vx = (-w[0] + w[1] - w[2] + w[3]) / 4.0f;

        X += vx * dt;
        Y += vy * dt;

        publish_odom(vx, vy);
    }

    void publish_odom(float vx, float vy) {
        nav_msgs::msg::Odometry odom;

        odom.header.stamp = now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = X;
        odom.pose.pose.position.y = Y;
        odom.pose.pose.position.z = 0.0;

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;

        odom_pub_->publish(odom);
    }

    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr rx_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdomNode>());
    rclcpp::shutdown();
    return 0;
}