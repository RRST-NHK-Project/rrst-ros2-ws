#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>
#include <limits>
#include <thread>

class KeyboardPublisher : public rclcpp::Node {
public:
    KeyboardPublisher() : Node("keyboard_num_publisher"), mode_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("target_point", 10);

        // Timer to publish periodically (10 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&KeyboardPublisher::publish_mode, this));

        // Separate thread for keyboard input
        input_thread_ = std::thread([this]() {
            while (rclcpp::ok()) {
                int value;
                std::cout << "Enter mode number (int): ";
                std::cin >> value;
                if (!std::cin) {
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    continue;
                }
                mode_ = value; // update mode
                RCLCPP_INFO(this->get_logger(), "Mode updated: %d", mode_);
            }
        });
    }

    ~KeyboardPublisher() {
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    void publish_mode() {
        auto msg = std_msgs::msg::Int32();
        msg.data = mode_;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread input_thread_;
    int mode_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
