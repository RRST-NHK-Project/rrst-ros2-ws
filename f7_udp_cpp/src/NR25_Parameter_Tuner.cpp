#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

class ParameterNode : public rclcpp::Node {
public:
    ParameterNode() : Node("parameter_node"), shoot_state(0), dribble_state(0) {
        params = {50, 50, 50, 50};
        publisher = this->create_publisher<std_msgs::msg::Int32MultiArray>("parameter_array", 10);

        running = true;
        publish_thread = std::thread(&ParameterNode::publish_parameters, this);
        input_thread = std::thread(&ParameterNode::handle_user_input, this);
    }

    ~ParameterNode() {
        running = false;
        if (publish_thread.joinable()) publish_thread.join();
        if (input_thread.joinable()) input_thread.join();
    }

private:
    std::vector<int> params;
    std::mutex param_mutex;
    std::atomic<int> shoot_state;
    std::atomic<int> dribble_state;
    std::atomic<bool> running;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher;
    std::thread publish_thread;
    std::thread input_thread;

    void publish_parameters() {
        while (running) {
            {
                std::lock_guard<std::mutex> lock(param_mutex); // shoot_state と dribble_state も含めてロック
                std_msgs::msg::Int32MultiArray msg;
                msg.data = {params[0], params[1], params[2], params[3], shoot_state.load(), dribble_state.load()};
                publisher->publish(msg);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void handle_user_input() {
        while (running) {
            std::string input;
            std::getline(std::cin, input);
            if (input.empty()) continue;

            std::lock_guard<std::mutex> lock(param_mutex);

            if (input == "s") {
                shoot_state = 1;
                show_parameters();  // 状態変更後に表示
                std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 0.5秒維持
                //shoot_state = 0;
                show_parameters();  // 状態リセット後に表示
            } 
            else if (input == "d") {
                dribble_state = 1;
                show_parameters();
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                //dribble_state = 0;
                show_parameters();
            } 
            else if (input == "show") {
                show_parameters();
            } 
            else {
                int idx, value;
                if (sscanf(input.c_str(), "%d %d", &idx, &value) == 2 && idx >= 0 && idx < 4 && value >= 0 && value <= 100) {
                    params[idx] = value;
                    show_parameters();
                } else {
                    std::cout << "Invalid input. Use: <index> <value> (0-3, 0-100), 's' (shoot), 'd' (dribble), or 'show'\n";
                }
            }
        }
    }

    void show_parameters() {
        std::cout << "\n=== Current Parameters ===\n";
        std::cout << "0: roller_speed_dribble_ab = " << params[0] << "\n";
        std::cout << "1: roller_speed_dribble_cd = " << params[1] << "\n";
        std::cout << "2: roller_speed_shoot_ab   = " << params[2] << "\n";
        std::cout << "3: roller_speed_shoot_cd   = " << params[3] << "\n";
        std::cout << "Shoot State: " << shoot_state.load() << "\n";
        std::cout << "Dribble State: " << dribble_state.load() << "\n";
        std::cout << "==========================\n";
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParameterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}