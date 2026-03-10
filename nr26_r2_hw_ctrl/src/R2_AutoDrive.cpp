#include "PacketController.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

PacketController pkt;

#define TARGET_DEVICE_ID 6
#define TX16NUM 24
#define PUBLISH_RATE_MS 50

class PurePursuitDrive : public rclcpp::Node {
public:
    PurePursuitDrive(uint8_t device_id)
        : Node("pure_pursuit_drive_" + std::to_string(device_id)), device_id_(device_id) {

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&PurePursuitDrive::cmd_vel_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(device_id_), 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&PurePursuitDrive::publish_timer, this));
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        float vx = msg->linear.x;  // 前後
        float vy = -msg->linear.y; // 左右
        float wz = msg->angular.z; // 回転

        // メカナム逆運動学
        v1 = vx + vy + wz; // 前左
        v3 = vx - vy - wz; // 前右
        v4 = vx - vy + wz; // 後左
        v2 = vx + vy - wz; // 後右

        // 向き補正
        v3 *= -1;
        v2 *= -1;

        // 正規化
        float max_v = std::max({fabsf(v1), fabsf(v2), fabsf(v3), fabsf(v4)});
        if (max_v < 1.0f)
            max_v = 1.0f;
        v1 /= max_v;
        v2 /= max_v;
        v3 /= max_v;
        v4 /= max_v;
    }

    void publish_timer() {
        pkt.setMD(MD5, static_cast<int16_t>(v1 * duty_max));
        pkt.setMD(MD6, static_cast<int16_t>(v2 * duty_max));
        pkt.setMD(MD7, static_cast<int16_t>(v3 * duty_max));
        pkt.setMD(MD8, static_cast<int16_t>(v4 * duty_max));

        std_msgs::msg::Int16MultiArray msg;
        msg.data = pkt.toVector();
        publisher_->publish(msg);
    }

    uint8_t device_id_;
    float duty_max = 100;
    float v1, v2, v3, v4;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PurePursuitDrive>(TARGET_DEVICE_ID);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}