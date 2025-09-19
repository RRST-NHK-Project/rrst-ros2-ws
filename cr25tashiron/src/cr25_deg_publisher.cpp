#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <string>
#include <thread>
#include <vector>

class MotorAnglePublisher : public rclcpp::Node {
public:
    MotorAnglePublisher() : Node("motor_angle_publisher") {
        // 初期角度
        motor_angles_ = {0, 0, 0, 0};

        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/motor_angles", 10);

        show_usage();

        running_ = true;
        publish_thread_ = std::thread(&MotorAnglePublisher::publish_loop, this);
        input_thread_ = std::thread(&MotorAnglePublisher::input_loop, this);
    }

    ~MotorAnglePublisher() {
        running_ = false;
        if (publish_thread_.joinable())
            publish_thread_.join();
        if (input_thread_.joinable())
            input_thread_.join();
    }

private:
    std::vector<int> motor_angles_;
    std::mutex mutex_;
    std::atomic<bool> running_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
    std::thread publish_thread_;
    std::thread input_thread_;

    void show_usage() {
        std::cout << "\n=== Motor Angle Publisher ===\n";
        std::cout << "4 motors (1-4) angle control\n";
        std::cout << "Usage:\n";
        std::cout << "  <motor_number> <angle> : set motor angle (1-4, -135~135)\n";
        std::cout << "  show                   : show current angles\n";
        std::cout << "===========================\n";
    }

    void publish_loop() {
        while (running_) {
            std_msgs::msg::Int32MultiArray msg;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                msg.data = motor_angles_;
            }
            publisher_->publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void input_loop() {
        while (running_) {
            std::string line;
            std::getline(std::cin, line);
            if (line.empty())
                continue;

            std::lock_guard<std::mutex> lock(mutex_);

            if (line == "show") {
                show_angles();
            } else {
                int motor_id, angle;
                if (sscanf(line.c_str(), "%d %d", &motor_id, &angle) == 2 &&
                    motor_id >= 1 && motor_id <= 4 && angle >= -135 && angle <= 135) {
                    motor_angles_[motor_id - 1] = angle;
                    std::cout << "Motor " << motor_id << " set to " << angle << "°\n";
                } else {
                    std::cout << "Invalid input. Format: <motor_number> <angle> (1-4, -135~135) or 'show'\n";
                }
            }
        }
    }

    void show_angles() {
        std::cout << "Current motor angles: [";
        for (size_t i = 0; i < motor_angles_.size(); ++i) {
            std::cout << motor_angles_[i];
            if (i != motor_angles_.size() - 1)
                std::cout << ", ";
        }
        std::cout << "]\n";
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorAnglePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
