#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

using namespace std::chrono_literals;

// ===== 設定 =====
#define TARGET_DEVICE_ID 6
#define TX16NUM 24
#define RX16NUM 17
#define PUBLISH_RATE_MS 20

constexpr float duty_max = 100.0f;

// オドメトリ用（XYのみ）
constexpr float WHEEL_RADIUS = 0.05f;

// 自動制御ゲイン
constexpr float KP_POS = 0.8f;
constexpr float GOAL_TOL = 0.05f; // [m]

// ==========================

class HardWareControl : public rclcpp::Node {
public:
    HardWareControl()
        : Node("mecanum_auto_manual") {

        data_.assign(TX16NUM, 0);

        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&HardWareControl::joy_callback, this, std::placeholders::_1));

        rx_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(TARGET_DEVICE_ID),
            10,
            std::bind(&HardWareControl::rx_callback, this, std::placeholders::_1));

        tx_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(TARGET_DEVICE_ID), 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&HardWareControl::publish, this));

        RCLCPP_INFO(get_logger(), "AUTO / MANUAL mecanum node (NO YAW)");
    }

private:
    // ===== 状態 =====
    float X = 0.0f;
    float Y = 0.0f;

    rclcpp::Time last_time;
    bool odom_init = false;

    bool auto_mode = false;
    bool last_option = false;

    // 目標座標（ここを変える）
    float target_x = 1.0f;
    float target_y = 1.0f;

    // ===== Joy =====
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

        // bool CROSS = msg->buttons[0];
        bool CIRCLE = msg->buttons[1];
        // bool TRIANGLE = msg->buttons[2];
        // bool SQUARE = msg->buttons[3];
        bool OPTION = msg->buttons[9];

        // AUTO / MANUAL 切替
        if (OPTION && !last_option) {
            auto_mode = !auto_mode;
            RCLCPP_WARN(
                get_logger(),
                "MODE: %s",
                auto_mode ? "AUTO" : "MANUAL");
        }
        last_option = OPTION;

        // =================================================================
        // CIRCLE:「棒ホールド機構」
        // ボタンを一回押すごとに2つのサーボモーターの角度状態を同時に変化させる
        // =================================================================
        // angle = 10のとき最下部までお仕込み
        // angle = 245のときマガジンに戻してる

        static int circle_pre = 0;
        static int t = 0;
        static int CIRCLE_PUSH_MAX = 4;

        if (CIRCLE == 1 && circle_pre == 0) {
            t = (t + 1) % CIRCLE_PUSH_MAX;
        }
        if (t == 0) {
            target_x = 1.0f;
            target_y = 0.0f;
        }
        if (t == 1) {
            target_x = 1.0f;
            target_y = 1.0f;
        }
        if (t == 2) {
            target_x = 0.0f;
            target_y = 1.0f;
        }

        if (t == 3) {
            target_x = 0.0f;
            target_y = 0.0f;
        }

        circle_pre = CIRCLE;

        if (!auto_mode) {
            manual_control(msg);
        }
    }

    void manual_control(const sensor_msgs::msg::Joy::SharedPtr msg) {

        float lx = -msg->axes[0];
        float ly = msg->axes[1];
        float r2 = (-msg->axes[5] + 1.0f) * 0.5f;

        if (r2 < 0.1f) {
            set_wheel(0, 0, 0, 0);
            return;
        }

        float rad = std::atan2(ly, lx);
        float vx = std::cos(rad) * r2;
        float vy = std::sin(rad) * r2;

        drive_mecanum(vx, vy);
    }

    // ===== AUTO制御（XYのみ）=====
    void auto_control() {

        float ex = target_x - X;
        float ey = target_y - Y;

        float dist = std::hypot(ex, ey);
        if (dist < GOAL_TOL) {
            set_wheel(0, 0, 0, 0);
            return;
        }

        float vx = std::clamp(ex * KP_POS, -1.0f, 1.0f);
        float vy = std::clamp(ey * KP_POS, -1.0f, 1.0f);

        drive_mecanum(vx, vy);

        RCLCPP_INFO(
            get_logger(),
            "[AUTO] X=%.2f Y=%.2f",
            X, Y);
    }

    // ===== メカナム逆運動学（回転なし）=====
    void drive_mecanum(float vx, float vy) {

        float v1 = vy - vx;  // 後左
        float v2 = -vy + vx; // 前右
        float v3 = -vy - vx; // 後右
        float v4 = vy + vx;  // 前左

        float max_v = std::max({fabsf(v1),
                                fabsf(v2),
                                fabsf(v3),
                                fabsf(v4),
                                1.0f});

        v1 /= max_v;
        v2 /= max_v;
        v3 /= max_v;
        v4 /= max_v;

        set_wheel(v1, v2, v3, v4);
    }

    void set_wheel(float v1, float v2, float v3, float v4) {
        data_[5] = static_cast<int16_t>(v1 * duty_max);
        data_[6] = static_cast<int16_t>(v2 * duty_max);
        data_[7] = static_cast<int16_t>(v3 * duty_max);
        data_[8] = static_cast<int16_t>(v4 * duty_max);
    }

    // ===== オドメトリ（XYのみ）=====
    void rx_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {

        if (msg->data.size() < RX16NUM)
            return;

        int16_t vel[4] = {
            msg->data[11],
            msg->data[12],
            msg->data[13],
            msg->data[14]};

        auto current_time = this->now();

        if (!odom_init) {
            last_time = current_time;
            odom_init = true;
            return;
        }

        float dt = (current_time - last_time).seconds();
        last_time = current_time;
        if (dt <= 0.0f)
            return;

        float w[4];
        for (int i = 0; i < 4; i++) {
            w[i] = vel[i] * 2.0f * M_PI / 60.0f * WHEEL_RADIUS;
        }

        float vy = (w[0] - w[1] - w[2] + w[3]) / 4.0f;
        float vx = (-w[0] + w[1] - w[2] + w[3]) / 4.0f;

        X += vx * dt;
        Y += vy * dt;
    }

    // ===== publish =====
    void publish() {

        if (auto_mode)
            auto_control();

        std_msgs::msg::Int16MultiArray msg;
        msg.data = data_;
        tx_pub_->publish(msg);
    }

    // ===== ROS =====
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr rx_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr tx_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<int16_t> data_;
};

// ===== main =====
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardWareControl>());
    rclcpp::shutdown();
    return 0;
}
