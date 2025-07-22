#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>

#define MC_PRINTF 1 // マイコン側のprintfを無効化・有効化(0 or 1)

std::vector<int16_t> data(19, 0);

class KeyboardInputHandler {
public:
    KeyboardInputHandler(rclcpp::Node::SharedPtr node)
        : node_(node), running_(true) {
        publisher_ = node_->create_publisher<std_msgs::msg::Int32MultiArray>("mr_swerve_drive", 10);
        input_thread_ = std::thread(&KeyboardInputHandler::keyboard_input_loop, this);
    }

    ~KeyboardInputHandler() {
        running_ = false;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    void keyboard_input_loop() {
        while (running_) {
            int index, value;
            std::cout << "Enter index (0-18) and value: ";

            if (!(std::cin >> index >> value)) {
                std::cerr << "Invalid input! Please enter two integers." << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                continue;
            }

            if (index < 0 || index >= 19) {
                std::cerr << "Invalid index! Enter a value between 0 and 18." << std::endl;
                continue;
            }

            // データ範囲のチェック
            if (index >= 1 && index <= 6) { // MD1~MD6
                value = std::clamp(value, -100, 100);
            } else if (index >= 7 && index <= 10) { // サーボ
                value = std::clamp(value, 0, 270);
            } else if (index >= 11 && index <= 18) { // TR
                value = (value != 0) ? 1 : 0;
            }

            data[index] = value;
            // data[0] = MC_PRINTF; // マイコン側のprintfを無効化・有効化(0 or 1)

            std::cout << "Updated data[" << index << "] = " << value << std::endl;
            print_data();

            // メッセージ作成・送信
            auto msg = std_msgs::msg::Int32MultiArray();
            msg.data.clear();
            for (auto &v : data) {
                msg.data.push_back(static_cast<int32_t>(v));
            }
            publisher_->publish(msg);
        }
    }

    void print_data() {
        std::cout << "Current data: [";
        for (size_t i = 0; i < data.size(); ++i) {
            std::cout << data[i];
            if (i < data.size() - 1)
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
    std::thread input_thread_;
    std::atomic<bool> running_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("mb_tester_node");

    std::string figletout = "figlet MB TESTER";
    int result = std::system(figletout.c_str());
    if (result != 0) {
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        std::cerr << "Please install 'figlet' with the following command:" << std::endl;
        std::cerr << "sudo apt install figlet" << std::endl;
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    }
    std::cout << "Keyboard input mode enabled. Enter index and value." << std::endl;

    KeyboardInputHandler keyboard_handler(node);

    rclcpp::spin(node); // ノードが回り続ける（ここでメインスレッドがブロック）

    rclcpp::shutdown();
    return 0;
}
