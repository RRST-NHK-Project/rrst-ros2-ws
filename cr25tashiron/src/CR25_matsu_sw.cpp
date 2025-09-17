/*
RRST-NHK-Project 2025
PS4コントローラーの入力を取得するサンプルプログラム
esp32マイコンにアクチュエータ指令を送るサンプルプログラム
*/

// 標準
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "actuator_msg/msg/actuator_msg.hpp"
#include <vector>

#define MC_PRINTF 0 // マイコン側のprintfを無効化・有効化(0 or 1)


std::vector<int16_t> out_data(25, 0); // マイコンに送信される配列"data"

class Angle_Listener : public rclcpp::Node {
public:
    Angle_Listener()
        : Node("angle_listener") {

        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "from_esp32_1", 10,
            std::bind(&Angle_Listener::angle_listener_callback, this,
                      std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("to_esp32_0", 10);

        RCLCPP_INFO(this->get_logger(),
                    "Angle Listener Node has been started.");
    }


private:   
    void angle_listener_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        bool sw1 = -msg->data[4]+1; // ボタンの状態を取得
        bool sw2 = -msg->data[5]+1; // ボタンの状態を取得
        bool sw3 = -msg->data[6]+1; // ボタンの状態を取得
       // bool angle3 = msg->data[10]; // ボタンの状態を取得
        //bool last_sw_3 = false;


        // if (last_sw_3 != sw3 && sw3 == true) {
        //     angle3 = 0;
        // }
       out_data[1] = sw1; 
         out_data[2] = sw2;
            out_data[3] = sw3;
        publish_data();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

    }
    void publish_data() {
        auto msg = std_msgs::msg::Int32MultiArray();
        msg.data.reserve(out_data.size());
        for (auto &v : out_data) {
            msg.data.push_back(static_cast<int32_t>(v));
        }
        publisher_->publish(msg);
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;

    int last_sw_1 = 0;
    int zero_offset_1 = 0;
    bool zero_set_1 = false;

};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto angle_listener = std::make_shared<Angle_Listener>();
    exec.add_node(angle_listener);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}