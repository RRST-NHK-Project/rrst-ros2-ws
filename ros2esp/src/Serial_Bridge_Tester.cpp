#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>

constexpr size_t TX16NUM = 8;

class KeyboardInputHandler {
public:
    KeyboardInputHandler(rclcpp::Node::SharedPtr node, uint8_t device_id)
        : node_(node), running_(true), device_id_(device_id) {

        publisher_ = node_->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(device_id_), 10);

        data_.assign(TX16NUM, 0);

        /* ★ 常時 publish 用タイマー（100ms = 10Hz） */
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&KeyboardInputHandler::timer_callback, this));

        input_thread_ = std::thread(&KeyboardInputHandler::keyboard_input_loop, this);
    }

    ~KeyboardInputHandler() {
        running_ = false;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    /* キーボード入力：data を更新するだけ */
    void keyboard_input_loop() {
        while (running_) {
            int index, value;
            std::cout << "Enter index (0-7) and value: ";

            if (!(std::cin >> index >> value)) {
                std::cerr << "Invalid input!" << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                continue;
            }

            if (index < 0 || index >= static_cast<int>(TX16NUM)) {
                std::cerr << "Index out of range (0-7)" << std::endl;
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                data_[index] = static_cast<int16_t>(value);
            }

            print_data();
        }
    }

    /* 周期 publish */
    void timer_callback() {
        std_msgs::msg::Int16MultiArray msg;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            msg.data = data_; // 必ず8要素
        }

        publisher_->publish(msg);
    }

    void print_data() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        std::cout << "TX DATA: [";
        for (size_t i = 0; i < data_.size(); ++i) {
            std::cout << data_[i];
            if (i + 1 < data_.size())
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<int16_t> data_;
    std::mutex data_mutex_;

    std::thread input_thread_;
    std::atomic<bool> running_;
    uint8_t device_id_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("serial_keyboard_tester");

    uint8_t device_id = 2; // SerialBridgeNode と一致
    KeyboardInputHandler keyboard_handler(node, device_id);

    RCLCPP_INFO(node->get_logger(),
                "Keyboard TX → serial_tx_%d (Int16 x 8, periodic publish)", device_id);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
