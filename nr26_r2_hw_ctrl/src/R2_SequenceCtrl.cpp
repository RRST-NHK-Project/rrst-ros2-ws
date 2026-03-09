/*
R2段差超えシーケンス（状態管理）
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
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

// 以下マイコンに合わせて設定
#define TARGET_DEVICE_ID 6 // 宛先マイコンのID
#define TX16NUM 24         // 送信データ数
#define RX16NUM 17         // 受信データ数

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

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
        mode_ = StepMode::STEP_UP;
        next_up(StepUpState::ALL_UP);
    }

    void start_step_down() {
        mode_ = StepMode::STEP_DOWN;
        next_down(StepDownState::FIRST_FORWARD);
    }

private:
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
        // ここに全てのサーボを上げる命令を書く
    }

    void front_down() {
        RCLCPP_INFO(get_logger(), "FRONT DOWN");
        // ここに前のサーボを下げる命令を書く
    }

    void rear_down() {
        RCLCPP_INFO(get_logger(), "REAR DOWN");
        // ここに後ろのサーボを下げる命令を書く
    }

    void stop_motion() {
        RCLCPP_INFO(get_logger(), "STOP");
        // ここに全てのモーターを止める命令を書く
    }

    void front_up() {
        RCLCPP_INFO(get_logger(), "FRONT UP");
        // ここに前のサーボを上げる命令を書く
    }

    void rear_up() {
        RCLCPP_INFO(get_logger(), "REAR UP");
        // ここに後ろのサーボを上げる命令を書く
    }

    void all_down() {
        RCLCPP_INFO(get_logger(), "ALL DOWN");
        // ここに全てのサーボを下げる命令を書く
    }

    void move_forward() {
        RCLCPP_INFO(get_logger(), "MOVE FORWARD");
        // ここに前進する命令を書く
    }

    void move_backward() {
        RCLCPP_INFO(get_logger(), "MOVE BACKWARD");
        // ここに後退する命令を書く
    }

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
            if ((now_time - state_start_time_).seconds() > 0.5)
                next_up(StepUpState::FRONT_DOWN);
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
            if ((now_time - state_start_time_).seconds() > 0.5)
                next_up(StepUpState::REAR_DOWN);
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
            if ((now_time - state_start_time_).seconds() > 0.5)
                next_up(StepUpState::DONE);
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
            next_down(StepDownState::FRONT_UP);
            break;

        case StepDownState::FRONT_UP:
            if (!state_executed_) {
                front_up();
                state_executed_ = true;
            }
            if ((now_time - state_start_time_).seconds() > 0.5)
                next_down(StepDownState::SECOND_FORWARD);
            break;

        case StepDownState::SECOND_FORWARD:
            if (!state_executed_) {
                move_forward();
                state_executed_ = true;
            }
            next_down(StepDownState::REAR_UP);
            break;

        case StepDownState::REAR_UP:
            if (!state_executed_) {
                rear_up();
                state_executed_ = true;
            }
            if ((now_time - state_start_time_).seconds() > 0.5)
                next_down(StepDownState::FINAL_FORWARD);
            break;

        case StepDownState::FINAL_FORWARD:
            if (!state_executed_) {
                move_forward();
                state_executed_ = true;
            }
            next_down(StepDownState::ALL_DOWN);
            break;

        case StepDownState::ALL_DOWN:
            if (!state_executed_) {
                all_down();
                state_executed_ = true;
            }
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

        // 配列を0で初期化
        data_.assign(TX16NUM, 0);

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

    static constexpr double WHEEL_RADIUS = 0.05; // [m]
    static constexpr double TREAD_X = 0.30;      // 前後半分 [m]
    static constexpr double TREAD_Y = 0.30;      // 左右半分 [m]

    float v1, v2, v3, v4; // 各メカナムホイールの速度指令値
                          // v1:第一象限, v2:第二象限, v3:第三象限, v4:第四象限

    // ===== オドメトリ状態 =====
    float X = 0, Y = 0, yaw_ = 0;
    int16_t vel_prev_[4]{0};
    bool vel_init_ = false;
    rclcpp::Time last_time_;

    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        float LS_X = -1 * msg->axes[0];
        float LS_Y = msg->axes[1];
        float RS_X = -1 * msg->axes[3];
        float RS_Y = msg->axes[4];

        bool CROSS = msg->buttons[0];
        // bool CIRCLE = msg->buttons[1];
        bool TRIANGLE = msg->buttons[2];
        // bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        bool L1 = msg->buttons[4];
        bool R1 = msg->buttons[5];

        // float L2_DIGITAL = (-1 * msg->axes[2] + 1) / 2;
        float R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2;

        // bool L2 = msg->buttons[6];
        // bool R2 = msg->buttons[7];

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        static bool last_up = false;
        static bool up_latch = false;
        static bool last_down = false;
        static bool down_latch = false;

        if (UP && !last_up) {
            up_latch = !up_latch;
            seq_->start_step_up();
        }

        if (DOWN && !last_down) {
            down_latch = !down_latch;
            seq_->start_step_down();
        }
        last_up = UP;
        last_down = DOWN;

        // 以降、配列data_を操作する
        float rad = atan2(LS_Y, LS_X);

        if (R2_DIGITAL >= 0.3) {

            float vx = cos(rad) * R2_DIGITAL;
            float vy = sin(rad) * R2_DIGITAL;

            // canのIDごとなのでつけ直して動かすとき注意!!!!!
            v2 = -vy + vx; // 前右
            v4 = vy + vx;  // 前左
            v1 = vy - vx;  // 後左
            v3 = -vy - vx; // 後右

            // float vx = cos(rad) * R2_DIGITAL;
            // float vy = sin(rad) * R2_DIGITAL;
            // float wz = RS_X * sp_yaw;

            // v1 = vy - vx - wz;
            // v2 = -vy + vx - wz;
            // v3 = -vy - vx + wz;
            // v4 = vy + vx + wz;

        } else if (RS_X >= deadzone || R1 == 1) {
            v2 = sp_yaw;
            v4 = sp_yaw;
            v1 = sp_yaw;
            v3 = sp_yaw;
        } else if (RS_X <= -deadzone || L1 == 1) {
            v2 = -sp_yaw;
            v4 = -sp_yaw;
            v1 = -sp_yaw;
            v3 = -sp_yaw;
        }

        else if (
            (fabsf(LS_X) <= deadzone) && (fabsf(LS_Y) <= deadzone) && (fabsf(RS_X) <= deadzone) && (fabsf(RS_Y) <= deadzone) && (R1 == 0) && (L1 == 0)) {
            v1 = 0.0;
            v2 = 0.0;
            v3 = 0.0;
            v4 = 0.0;
        }

        // 正規化(これで全方向安定した速度出せる)
        float max_v = std::max({fabsf(v1), fabsf(v2), fabsf(v3), fabsf(v4), 1.0f});

        v1 /= max_v;
        v2 /= max_v;
        v3 /= max_v;
        v4 /= max_v;

        // if (up_latch == true) {
        //     Action::for_up(data_);
        // } else if (up_latch == false) {
        //     Action::for_down(data_);
        // }
        // if (down_latch == true) {
        //     Action::back_up(data_);
        // } else if (down_latch == false) {
        //     Action::back_down(data_);
        // }

        // if (CROSS) {
        //     Action::all_down(data_);
        //     up_latch = false;
        //     down_latch = false;
        // }

        // if (TRIANGLE) {
        //     Action::all_up(data_);
        //     up_latch = true;
        //     down_latch = true;
        // }

        // 2026/02/14, 7,8,9,10を5,6,7,8に変更
        data_[5] = static_cast<int16_t>(v1 * duty_max);
        data_[6] = static_cast<int16_t>(v2 * duty_max);
        data_[7] = static_cast<int16_t>(v3 * duty_max);
        data_[8] = static_cast<int16_t>(v4 * duty_max);
    }

    void publisher_timer_callback() {
        std_msgs::msg::Int16MultiArray msg;

        msg.data = data_;

        publisher_->publish(msg);
        // print_data();
    }

    void print_data() {
        std::cout << "TX DATA: [";
        for (size_t i = 0; i < data_.size(); ++i) {
            std::cout << data_[i];
            if (i + 1 < data_.size())
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }

    void
    sensor_callback(
        const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        // 最低限：サイズチェック
        if (msg->data.size() < RX16NUM) {
            RCLCPP_WARN(this->get_logger(),
                        "serial_rx_%d: data too short (%zu)",
                        device_id_, msg->data.size());
            return;
        }

        //  int16_t ENC1 = msg->data[1];
        //  int16_t ENC2 = msg->data[2];
        //  int16_t ENC3 = msg->data[3];
        //  int16_t ENC4 = msg->data[4];
        //  int16_t ENC5 = msg->data[5];
        //  int16_t ENC6 = msg->data[6];
        //  int16_t ENC7 = msg->data[7];
        //  int16_t ENC8 = msg->data[8];

        // int16_t SW1 = msg->data[9];
        // int16_t SW2 = msg->data[10];
        // int16_t SW3 = msg->data[11];
        // int16_t SW4 = msg->data[12];
        // int16_t SW5 = msg->data[13];
        // int16_t SW6 = msg->data[14];
        // int16_t SW7 = msg->data[15];
        // int16_t SW8 = msg->data[16];

        // int16_t angle1 = msg->data[7];
        // int16_t angle2 = msg->data[8];
        // int16_t angle3 = msg->data[9];
        // int16_t angle4 = msg->data[10];

        int16_t vel[4];
        vel[0] = msg->data[11];
        vel[1] = msg->data[12];
        vel[2] = msg->data[13];
        vel[3] = msg->data[14];

        // 以降、受信データを使った処理を記述
        if (!vel_init_) {

            last_time_ = now();
            vel_init_ = true;
            return;
        }
        double dt = (now() - last_time_).seconds();
        last_time_ = now();
        if (dt <= 0)
            return;

        float w[4];
        for (int i = 0; i < 4; i++) {
            float omega = vel[i] * 2.0f * M_PI / 60.0f; // [rad/s]
            w[i] = omega * WHEEL_RADIUS;
        }

        float vy = (w[0] - w[1] - w[2] + w[3]) / 4.0f;
        float vx = (-w[0] + w[1] - w[2] + w[3]) / 4.0f;
        float wz = (-w[0] + w[1] + w[2] - w[3]) / (4.0f * (TREAD_X + TREAD_Y));

        X += (vx * cos(yaw_) - vy * sin(yaw_)) * dt;
        Y += (vx * sin(yaw_) + vy * cos(yaw_)) * dt;
        yaw_ += wz * dt;
        yaw_ = atan2(sin(yaw_), cos(yaw_));

        RCLCPP_INFO(get_logger(),
                    "X: %.2f m, Y: %.2f m, Yaw: %.2f deg",
                    X, Y, yaw_ * 180.0 / M_PI);

        // 受信データ処理ここまで
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