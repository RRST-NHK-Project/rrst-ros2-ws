/*
R2ハンド制御
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

// 標準
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

// 自作
#include "include/PacketController.hpp"
PacketController pkt;

// マイコン設定
#define TARGET_DEVICE_ID 7 // 宛先マイコンのID
#define TX16NUM 24         // 送信データ数
#define PUBLISH_RATE_MS 20 // publish周期(ms)

using namespace std::chrono_literals;

class HardWareControl : public rclcpp::Node {
public:
    HardWareControl(uint8_t device_id)
        : Node("hardware_control_" + std::to_string(device_id)),
          device_id_(device_id) {
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(device_id_), 10);

        // GUIから直接状態が送られる場合の互換経路
        state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "r2/task_state", 10,
            std::bind(&HardWareControl::task_state_callback, this, std::placeholders::_1));

        // r2_plannerが管理する現在状態（[state, color, cell, mode]）
        auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        status_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "r2/task_status", status_qos,
            std::bind(&HardWareControl::task_status_callback, this, std::placeholders::_1));

        status_text_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/r2/task_status_text", status_qos,
            std::bind(&HardWareControl::task_status_text_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&HardWareControl::publish_packet, this));

        apply_state_to_packet(current_planner_state_);
        RCLCPP_INFO(get_logger(), "serial_tx_%d started.", device_id_);
    }

private:
    static constexpr int32_t STATE_KFS_HAND_INIT = 6;
    static constexpr int32_t STATE_KFS_PICK_WAITING = 7;
    static constexpr int32_t STATE_PICK_UP = 8;
    static constexpr int32_t STATE_PICK_MIDDLE = 9;
    static constexpr int32_t STATE_PICK_DOWN = 10;
    static constexpr int32_t STATE_KFS_HOLD = 11;
    static constexpr int32_t STATE_KFS_MOVE = 12;
    static constexpr int32_t STATE_TTR_SHOOT_MIDDLE = 13;

    enum class TargetHeight {
        UP,
        MIDDLE,
        DOWN
    };

    int32_t current_planner_state_ = 0;
    std::string current_state_name_ = "";
    TargetHeight target_height_ = TargetHeight::DOWN;

    uint8_t device_id_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_text_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    static int32_t state_code_from_name(const std::string &state_name) {
        if (state_name == "KFS_HAND_INIT")
            return STATE_KFS_HAND_INIT;
        if (state_name == "KFS_PICK_WAITING")
            return STATE_KFS_PICK_WAITING;
        if (state_name == "PICK_UP")
            return STATE_PICK_UP;
        if (state_name == "PICK_MIDDLE")
            return STATE_PICK_MIDDLE;
        if (state_name == "PICK_DOWN")
            return STATE_PICK_DOWN;
        if (state_name == "KFS_HOLD")
            return STATE_KFS_HOLD;
        if (state_name == "KFS_MOVE")
            return STATE_KFS_MOVE;
        if (state_name == "TTR_SHOOT_MIDDLE")
            return STATE_TTR_SHOOT_MIDDLE;
        return -1;
    }

    static std::string parse_state_name(const std::string &status_text) {
        const std::string key = "state=";
        const auto pos = status_text.find(key);
        if (pos == std::string::npos) {
            return "";
        }

        const auto begin = pos + key.size();
        const auto end = status_text.find(' ', begin);
        if (end == std::string::npos) {
            return status_text.substr(begin);
        }
        return status_text.substr(begin, end - begin);
    }

    void reset_hand_outputs() {
        pkt.setServo(SERVO1, 0);
        pkt.setTR(TR1, false);
        pkt.setTR(TR2, false);
    }

    void set_home_values() {
        pkt.setServo(SERVO1, 0);
        pkt.setTR(TR1, false); // シリンダー縮める
        pkt.setTR(TR2, false); // ハンド開く
    }

    void set_ready_values() {
        if (target_height_ == TargetHeight::UP) {
            pkt.setServo(SERVO1, 45);
        } else {
            pkt.setServo(SERVO1, 90);
        }
        pkt.setTR(TR1, true); // シリンダー伸ばす
        pkt.setTR(TR2, false);
    }

    void set_pick_values() {
        pkt.setTR(TR1, true);
        pkt.setTR(TR2, true); // ハンド閉じる
    }

    void set_hold_values() {
        pkt.setTR(TR1, true);
        pkt.setTR(TR2, true);
    }

    void set_moving_values() {
        pkt.setTR(TR1, true);
        pkt.setTR(TR2, true);
    }

    void set_shoot_values() {
        pkt.setTR(TR1, true);
        pkt.setTR(TR2, true);
    }

    void apply_state_to_packet(int32_t state_code) {
        // 状態ごとに明示的に値を作り直し、残留値を防ぐ
        reset_hand_outputs();

        switch (state_code) {
        case STATE_KFS_HAND_INIT:
            set_home_values();
            break;
        case STATE_KFS_PICK_WAITING:
            set_ready_values();
            break;
        case STATE_PICK_UP:
            target_height_ = TargetHeight::UP;
            set_ready_values();
            set_pick_values();
            break;
        case STATE_PICK_MIDDLE:
            target_height_ = TargetHeight::MIDDLE;
            set_ready_values();
            set_pick_values();
            break;
        case STATE_PICK_DOWN:
            target_height_ = TargetHeight::DOWN;
            set_ready_values();
            set_pick_values();
            break;
        case STATE_KFS_HOLD:
            set_hold_values();
            break;
        case STATE_KFS_MOVE:
            set_moving_values();
            break;
        case STATE_TTR_SHOOT_MIDDLE:
            set_shoot_values();
            break;
        default:
            // ハンド非関連状態では安全側
            set_home_values();
            break;
        }
    }

    void task_state_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        const int32_t next_state = msg->data;
        if (next_state == current_planner_state_) {
            return;
        }

        current_planner_state_ = next_state;
        apply_state_to_packet(current_planner_state_);
        RCLCPP_INFO(get_logger(), "task_state -> %ld", static_cast<long>(current_planner_state_));
    }

    void task_status_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.empty()) {
            return;
        }

        const int32_t next_state = msg->data[0];
        if (next_state == current_planner_state_) {
            return;
        }

        current_planner_state_ = next_state;
        apply_state_to_packet(current_planner_state_);
        RCLCPP_INFO(get_logger(), "task_status.state -> %ld", static_cast<long>(current_planner_state_));
    }

    void task_status_text_callback(const std_msgs::msg::String::SharedPtr msg) {
        const std::string next_state_name = parse_state_name(msg->data);
        if (next_state_name.empty() || next_state_name == current_state_name_) {
            return;
        }

        current_state_name_ = next_state_name;
        current_planner_state_ = state_code_from_name(current_state_name_);
        apply_state_to_packet(current_planner_state_);

        RCLCPP_INFO(get_logger(), "task_status_text.state -> %s", current_state_name_.c_str());
    }

    void publish_packet() {
        std_msgs::msg::Int16MultiArray msg;
        msg.data = pkt.toVector();
        publisher_->publish(msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::string figletout = "figlet R2_HandCtrl";
    int result = std::system(figletout.c_str());
    if (result != 0) {
        std::cerr << "Please install 'figlet' with: sudo apt install figlet\n";
    }

    auto hardware_control = std::make_shared<HardWareControl>(TARGET_DEVICE_ID);
    rclcpp::spin(hardware_control);
    rclcpp::shutdown();
    return 0;
}