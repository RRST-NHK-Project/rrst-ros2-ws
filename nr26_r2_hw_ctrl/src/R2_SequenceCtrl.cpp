/*
R2段差超えシーケンス（状態管理）
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

/*
R2について以下の不具合を確認しています。
L1、R1で回転しようとすると前進、後進してしまう
マイコンのモードによりロボマスとエンコーダの取得のみしかできずソレノイドの駆動ができない（新規モードの作成が必要）
シーケンス内の前進において前進せずその場で回転してしまう
*/

// 標準
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

// ROS　
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

// 自作
#include "include/PacketController.hpp"
PacketController pkt;

// 以下マイコンに合わせて設定
#define TARGET_DEVICE_ID 6 // 宛先マイコンのID
#define TX16NUM 24         // 送信データ数
#define RX16NUM 17         // 受信データ数

#define PUBLISH_RATE_MS 50 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

using namespace std::chrono_literals;

class SequenceControl;
class HardWareControl;

// 状態を管理しながらシーケンス制御
class SequenceControl : public rclcpp::Node {
public:
    SequenceControl() : Node("sequence_ctrl_node") {
        timer_ = this->create_wall_timer(
            10ms,
            std::bind(&SequenceControl::loop, this));
    }

    // トリガー関数

    void start_step_up() {
        if (mode_ != StepMode::NONE) {
            RCLCPP_WARN(get_logger(), "Sequence busy. STEP_UP ignored.");
            return; // 実行中なら無視
        }
        mode_ = StepMode::STEP_UP;
        next_up(StepUpState::ALL_UP);
    }

    void start_step_down() {
        if (mode_ != StepMode::NONE) {
            RCLCPP_WARN(get_logger(), "Sequence busy. STEP_DOWN ignored.");
            return; // 実行中なら無視
        }
        mode_ = StepMode::STEP_DOWN;
        next_down(StepDownState::FIRST_FORWARD);
    }

    // シーケンス実行中の判定
    bool is_busy() const {
        return mode_ != StepMode::NONE;
    }

private:
    // 以下シーケンス内で使用する変数
    //  待機時間（要調整）
    static constexpr double up_first_forward_wait = 1.0;
    static constexpr double up_second_forward_wait = 1.0;
    static constexpr double up_final_forward_wait = 1.0;
    static constexpr double down_first_forward_wait = 1.0;
    static constexpr double down_second_forward_wait = 1.0;
    static constexpr double down_final_forward_wait = 1.0;

    // 速度関連
    static constexpr int forward_speed = 10;

    // モードの管理
    enum class StepMode {
        NONE,
        STEP_UP,
        STEP_DOWN
    };

    // 状態管理（上り）
    enum class StepUpState {
        IDLE,
        ALL_UP,
        FIRST_FORWARD,
        FRONT_DOWN,
        SECOND_FORWARD,
        REAR_DOWN,
        FINAL_FORWARD,
        DONE
    };

    // 状態管理（下り）
    enum class StepDownState {
        IDLE,
        FIRST_FORWARD,
        FRONT_UP,
        SECOND_FORWARD,
        REAR_UP,
        FINAL_FORWARD,
        ALL_DOWN,
        DONE
    };

    StepMode mode_ = StepMode::NONE;

    StepUpState state_up_ = StepUpState::IDLE;
    StepDownState state_down_ = StepDownState::IDLE;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time state_start_time_;
    bool state_executed_ = false; // 各状態での処理の実行状況を保存

    // 状態遷移関数
    void next_up(StepUpState next) {
        state_start_time_ = this->now();
        state_up_ = next;
        state_executed_ = false;
    }

    void next_down(StepDownState next) {
        state_start_time_ = this->now();
        state_down_ = next;
        state_executed_ = false;
    }

    // 機構関数
    void all_up() {
        RCLCPP_INFO(get_logger(), "ALL UP");
        pkt.setTR(TR1, 1);
        pkt.setTR(TR2, 1);
    }

    void front_down() {
        RCLCPP_INFO(get_logger(), "FRONT DOWN");
        pkt.setTR(TR1, 0);
    }

    void rear_down() {
        RCLCPP_INFO(get_logger(), "REAR DOWN");
        pkt.setTR(TR2, 0);
    }

    void stop_motion() {
        RCLCPP_INFO(get_logger(), "STOP");
        pkt.setTR(TR1, 0);
        pkt.setTR(TR2, 0);
        pkt.setMD(MD5, forward_speed);
        pkt.setMD(MD6, forward_speed);
        pkt.setMD(MD7, forward_speed);
        pkt.setMD(MD8, forward_speed);
    }

    void front_up() {
        RCLCPP_INFO(get_logger(), "FRONT UP");
        pkt.setTR(TR1, 1);
    }

    void rear_up() {
        RCLCPP_INFO(get_logger(), "REAR UP");
        pkt.setTR(TR2, 1);
    }

    void all_down() {
        RCLCPP_INFO(get_logger(), "ALL DOWN");
        pkt.setTR(TR1, 0);
        pkt.setTR(TR2, 0);
    }

    void move_forward() {
        RCLCPP_INFO(get_logger(), "MOVE FORWARD");
        pkt.setMD(MD5, forward_speed);
        pkt.setMD(MD6, forward_speed);
        pkt.setMD(MD7, forward_speed);
        pkt.setMD(MD8, forward_speed);
    }

    // void move_backward() {
    //     RCLCPP_INFO(get_logger(), "MOVE BACKWARD");
    // }

    // 段差超えシーケンス（上り）
    void step_up_sequence() {
        auto now_time = now();
        switch (state_up_) {
        case StepUpState::IDLE:
            break;

        case StepUpState::ALL_UP:
            if (!state_executed_) {
                all_up();
                state_executed_ = true;
            }
            next_up(StepUpState::FIRST_FORWARD);
            break;

        case StepUpState::FIRST_FORWARD:
            if (!state_executed_) {
                move_forward();
                state_executed_ = true;
            }
            if ((now_time - state_start_time_).seconds() > up_first_forward_wait) {
                next_up(StepUpState::FRONT_DOWN);
            }
            break;

        case StepUpState::FRONT_DOWN:
            if (!state_executed_) {
                front_down();
                state_executed_ = true;
            }
            next_up(StepUpState::SECOND_FORWARD);
            break;

        case StepUpState::SECOND_FORWARD:
            if (!state_executed_) {
                move_forward();
                state_executed_ = true;
            }
            if ((now_time - state_start_time_).seconds() > up_second_forward_wait) {
                next_up(StepUpState::REAR_DOWN);
            }
            break;

        case StepUpState::REAR_DOWN:
            if (!state_executed_) {
                rear_down();
                state_executed_ = true;
            }
            next_up(StepUpState::FINAL_FORWARD);
            break;

        case StepUpState::FINAL_FORWARD:
            if (!state_executed_) {
                move_forward();
                state_executed_ = true;
            }
            if ((now_time - state_start_time_).seconds() > up_final_forward_wait) {
                next_up(StepUpState::DONE);
            }
            break;

        case StepUpState::DONE:
            if (!state_executed_) {
                stop_motion();
                state_executed_ = true;
            }
            mode_ = StepMode::NONE;
            state_up_ = StepUpState::IDLE;
            break;
        }
    }

    // 段差超えシーケンス（下り）
    void step_down_sequence() {
        auto now_time = now();
        switch (state_down_) {
        case StepDownState::IDLE:
            break;

        case StepDownState::FIRST_FORWARD:
            if (!state_executed_) {
                move_forward();
                state_executed_ = true;
            }
            if ((now_time - state_start_time_).seconds() > down_first_forward_wait) {
                next_down(StepDownState::FRONT_UP);
            }
            break;

        case StepDownState::FRONT_UP:
            if (!state_executed_) {
                front_up();
                state_executed_ = true;
            }
            next_down(StepDownState::SECOND_FORWARD);
            break;

        case StepDownState::SECOND_FORWARD:
            if (!state_executed_) {
                move_forward();
                state_executed_ = true;
            }
            if ((now_time - state_start_time_).seconds() > down_second_forward_wait) {
                next_down(StepDownState::REAR_UP);
            }
            break;

        case StepDownState::REAR_UP:
            if (!state_executed_) {
                rear_up();
                state_executed_ = true;
            }
            next_down(StepDownState::FINAL_FORWARD);
            break;

        case StepDownState::FINAL_FORWARD:
            if (!state_executed_) {
                move_forward();
                state_executed_ = true;
            }
            if ((now_time - state_start_time_).seconds() > down_final_forward_wait) {
                next_down(StepDownState::ALL_DOWN);
            }
            break;

        case StepDownState::ALL_DOWN:
            if (!state_executed_) {
                all_down();
                state_executed_ = true;
            }
            if ((now_time - state_start_time_).seconds() > 0.5)
                next_down(StepDownState::DONE);
            break;

        case StepDownState::DONE:
            if (!state_executed_) {
                stop_motion();
                state_executed_ = true;
            }
            mode_ = StepMode::NONE;
            state_down_ = StepDownState::IDLE;
            break;
        }
    }

    void loop() {

        switch (mode_) {

        case StepMode::NONE:
            break;

        case StepMode::STEP_UP:
            step_up_sequence();
            break;

        case StepMode::STEP_DOWN:
            step_down_sequence();
            break;
        }
    }
};

class HardWareControl : public rclcpp::Node {
public:
    HardWareControl(uint8_t device_id, std::shared_ptr<SequenceControl> seq)
        : Node("hardware_control_" + std::to_string(device_id)),
          device_id_(device_id),
          seq_(seq) {

        // joyノードのSubscribe
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&HardWareControl::ps4_listener_callback, this, std::placeholders::_1));

        // seial_bridgeへpublish
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(device_id_), 10);

        // timer_callbackを呼び出すタイマーを作成
        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&HardWareControl::publisher_timer_callback, this));

        sensor_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(device_id_),
            10,
            std::bind(&HardWareControl::sensor_callback,
                      this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "serial_tx_%d started.", device_id_);
    }

private:
    // 定数・変数
    float duty_max = 100;
    float for_speed = 50;
    float back_speed = 50;
    float sp_yaw = 0.5;
    float deadzone = 0.3; // adjust DS4 deadzone

    float v1, v2, v3, v4; // 各メカナムホイールの速度指令値
                          // v1:第一象限, v2:第二象限, v3:第三象限, v4:第四象限

    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (seq_->is_busy()) {
            return;
        }

        float LS_X = -msg->axes[0]; // 左右
        float LS_Y = msg->axes[1];  // 前後
        float RS_X = -msg->axes[3]; // 回転

        float R2 = (-msg->axes[5] + 1) / 2;

        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        bool L1 = msg->buttons[4];
        bool R1 = msg->buttons[5];

        static bool last_up = false;
        static bool last_down = false;

        if (UP && !last_up) {
            seq_->start_step_up();
        }

        if (DOWN && !last_down) {
            seq_->start_step_down();
        }

        last_up = UP;
        last_down = DOWN;

        if (fabsf(LS_X) < deadzone)
            LS_X = 0;
        if (fabsf(LS_Y) < deadzone)
            LS_Y = 0;
        if (fabsf(RS_X) < deadzone)
            RS_X = 0;

        float vx = -LS_Y * R2;    // 前後
        float vy = LS_X * R2;     // 左右
        float wz = RS_X * sp_yaw; // 回転

        // 逆運動学
        v1 = vx + vy + wz; // 前左
        v3 = vx - vy - wz; // 前右
        v4 = vx - vy + wz; // 後左
        v2 = vx + vy - wz; // 後右

        // 向き補正
        v3 *= -1;
        v2 *= -1;

        // 回転
        if (R1) {
            v1 = sp_yaw;
            v2 = -sp_yaw;
            v3 = -sp_yaw;
            v4 = sp_yaw;
        }

        if (L1) {
            v1 = -sp_yaw;
            v2 = sp_yaw;
            v3 = sp_yaw;
            v4 = -sp_yaw;
        }

        // 正規化
        float max_v = std::max(
            std::max(fabsf(v1), fabsf(v2)),
            std::max(fabsf(v3), fabsf(v4)));

        if (max_v < 1.0f)
            max_v = 1.0f;

        v1 /= max_v;
        v2 /= max_v;
        v3 /= max_v;
        v4 /= max_v;

        // 出力
        pkt.setMD(MD5, static_cast<int16_t>(v1 * duty_max));
        pkt.setMD(MD6, static_cast<int16_t>(v2 * duty_max));
        pkt.setMD(MD7, static_cast<int16_t>(v3 * duty_max));
        pkt.setMD(MD8, static_cast<int16_t>(v4 * duty_max));
    }

    void publisher_timer_callback() {
        std_msgs::msg::Int16MultiArray msg;

        // msg.data = data_;
        msg.data = pkt.toVector();

        publisher_->publish(msg);
        // print_data();
    }

    void print_data() {
        const auto &pkt_data = pkt.data_;
        std::cout << "TX DATA: [";
        for (size_t i = 0; i < pkt_data.size(); ++i) {
            std::cout << pkt_data[i];
            if (i + 1 < pkt_data.size())
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }

    void sensor_callback(
        const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        if (msg->data.size() < RX16NUM) {
            RCLCPP_WARN(this->get_logger(),
                        "serial_rx_%d: data too short (%zu)",
                        device_id_, msg->data.size());
            return;
        }
    }

    uint8_t device_id_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<int16_t> data_;

    // シーケンスの管理に使う
    std::shared_ptr<SequenceControl> seq_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet R2_SequenceCtrl";
    int result = std::system(figletout.c_str());
    if (result != 0) {
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
        std::cerr << "Please install 'figlet' with the following command:"
                  << std::endl;
        std::cerr << "sudo apt install figlet" << std::endl;
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
    }

    rclcpp::executors::MultiThreadedExecutor exec;

    auto sequence_control = std::make_shared<SequenceControl>();
    auto hardware_control = std::make_shared<HardWareControl>(TARGET_DEVICE_ID, sequence_control);
    exec.add_node(hardware_control);
    exec.add_node(sequence_control);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}