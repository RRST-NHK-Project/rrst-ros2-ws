/*
ESC用Serial_Bridgeノードのホスト側開発フレーム
*/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>

// 以下ESCマイコンに合わせて設定
#define TX16NUM 24
#define PUBLISH_RATE_MS 20

namespace {
    constexpr float kTargetVelocityScale = 0.1f; // int16 <-> rad/s
    constexpr float kTargetAngleScale = 0.1f;    // int16 <-> deg
    constexpr float kVoltageLimitScale = 0.1f;   // int16 <-> V
    constexpr double kTau = 6.28318530717958647692;

    constexpr std::size_t kCmdEnable = 1;
    constexpr std::size_t kCmdMode = 2;
    constexpr std::size_t kCmdTargetVelocity = 3;
    constexpr std::size_t kCmdTargetAngle = 4;
    constexpr std::size_t kCmdVoltageLimit = 5;

    constexpr std::size_t kRxAngle = 1;
    constexpr std::size_t kRxVelocity = 2;
    constexpr std::size_t kRxTarget = 3;
    constexpr std::size_t kRxMode = 4;
    constexpr std::size_t kRxVoltageLimit = 5;

    int16_t clamp_i16(long value) {
        if (value > 32767) {
            return 32767;
        }
        if (value < -32768) {
            return -32768;
        }
        return static_cast<int16_t>(value);
    }

    double rpm_to_rad_per_sec(double rpm) {
        return rpm * kTau / 60.0;
    }

    double rad_per_sec_to_rpm(double rad_per_sec) {
        return rad_per_sec * 60.0 / kTau;
    }
} // namespace

class HardWareControl : public rclcpp::Node {
public:
    HardWareControl()
        : Node("esc_ctrl_dev_frame") {

        tx_device_id_ = this->declare_parameter<int>("device_id", 153);
        rx_device_id_ = this->declare_parameter<int>("rx_device_id", tx_device_id_);
        joy_topic_ = this->declare_parameter<std::string>("joy_topic", "joy");
        publish_rate_ms_ = this->declare_parameter<int>("tx_period_ms", PUBLISH_RATE_MS);
        enable_on_start_ = this->declare_parameter<bool>("enable_on_start", true);
        start_mode_ = this->declare_parameter<int>("start_mode", 0);
        start_velocity_rpm_ = this->declare_parameter<double>("start_velocity_rpm", 0.0);
        start_angle_deg_ = this->declare_parameter<double>("start_angle_deg", 0.0);
        start_voltage_limit_ = this->declare_parameter<double>("start_voltage_limit", 12.0);
        show_rx_logs_ = this->declare_parameter<bool>("show_rx_logs", true);

        // 配列を0で初期化
        data_.assign(TX16NUM, 0);
        /*
        ESCマイコンに送信される配列"data_"
        | data[n] | 詳細 | 単位/範囲 |
        | ---- | ---- | ---- |
        | data[0] | debug | 予約 |
        | data[1] | enable | 0 or 1 |
        | data[2] | mode | 0: velocity / 1: angle |
        | data[3] | target_velocity | rad/s / 0.1scale |
        | data[4] | target_angle | deg / 0.1scale |
        | data[5] | voltage_limit | V / 0.1scale |
        | data[6..23] | reserve | 予約 |
        */

        data_[kCmdEnable] = enable_on_start_ ? 1 : 0;
        data_[kCmdMode] = (start_mode_ == 1) ? 1 : 0;
        data_[kCmdTargetVelocity] =
            clamp_i16(std::lround(rpm_to_rad_per_sec(start_velocity_rpm_) / kTargetVelocityScale));
        data_[kCmdTargetAngle] =
            clamp_i16(std::lround(start_angle_deg_ / kTargetAngleScale));
        data_[kCmdVoltageLimit] =
            clamp_i16(std::lround(start_voltage_limit_ / kVoltageLimitScale));

        // joyノードのSubscribe
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            joy_topic_, 10,
            std::bind(&HardWareControl::ps4_listener_callback, this, std::placeholders::_1));

        // serial_bridgeへpublish
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(tx_device_id_), 10);

        // timer_callbackを呼び出すタイマーを作成
        timer_ = create_wall_timer(
            std::chrono::milliseconds(std::max(1, publish_rate_ms_)),
            std::bind(&HardWareControl::publisher_timer_callback, this));

        sensor_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(rx_device_id_),
            10,
            std::bind(&HardWareControl::sensor_callback,
                      this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "esc_ctrl_dev_frame started.");
        RCLCPP_INFO(get_logger(), "topics: joy=%s tx=serial_tx_%d rx=serial_rx_%d",
                    joy_topic_.c_str(), tx_device_id_, rx_device_id_);
        RCLCPP_INFO(get_logger(),
                    "Edit ps4_listener_callback() and sensor_callback() in Host style.");
    }

private:
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        // float LS_X = -1 * msg->axes[0];
        // float LS_Y = msg->axes[1];
        // float RS_X = -1 * msg->axes[3];
        // float RS_Y = msg->axes[4];

        // bool CROSS = msg->buttons[0];
        // bool CIRCLE = msg->buttons[1];
        // bool TRIANGLE = msg->buttons[2];
        // bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        // bool UP = msg->axes[7] == 1.0;
        // bool DOWN = msg->axes[7] == -1.0;

        // bool L1 = msg->buttons[4];
        // bool R1 = msg->buttons[5];

        // float L2_DIGITAL = (-1 * msg->axes[2] + 1) / 2;
        // float R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2;

        // bool L2 = msg->buttons[6];
        // bool R2 = msg->buttons[7];

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        // static bool last_option = false;
        // static bool option_latch = false;

        // static bool last_share = false;
        // static bool share_latch = false;

        // 以降、配列data_を操作する
        // ESCの速度指令をRPMで考える場合でも、送信値はrad/sスケールで入れる。
        // 例:
        // data_[kCmdEnable] = CROSS ? 1 : 0;
        // data_[kCmdMode] = 0;
        // data_[kCmdTargetVelocity] =
        //     clamp_i16(std::lround(rpm_to_rad_per_sec(LS_Y * 3000.0) / kTargetVelocityScale));
        // data_[kCmdTargetAngle] = 0;
        // data_[kCmdVoltageLimit] =
        //     clamp_i16(std::lround(12.0 / kVoltageLimitScale));
        // if (CIRCLE) {
        //     data_[kCmdMode] = 1;
        //     data_[kCmdTargetVelocity] = 0;
        //     data_[kCmdTargetAngle] =
        //         clamp_i16(std::lround(RS_Y * 180.0 / kTargetAngleScale));
        // }

        // デバッグ用
        // RCLCPP_INFO(
        //     get_logger(),
        //     "cmd=[enable=%d mode=%d vel=%d ang=%d vlim=%d]",
        //     data_[kCmdEnable], data_[kCmdMode], data_[kCmdTargetVelocity],
        //     data_[kCmdTargetAngle], data_[kCmdVoltageLimit]);

        (void)msg;

        // 配列操作ここまで
    }

    // publish
    void publisher_timer_callback() {
        std_msgs::msg::Int16MultiArray msg;

        msg.data = data_;

        publisher_->publish(msg);
    }

    void sensor_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        if (msg->data.size() <= kRxVoltageLimit) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                                 "serial_rx frame too short: %zu", msg->data.size());
            return;
        }

        // ESCからの受信値例
        int16_t angle_raw = msg->data[kRxAngle];
        int16_t velocity_raw = msg->data[kRxVelocity];
        int16_t target_raw = msg->data[kRxTarget];
        int16_t mode = msg->data[kRxMode];
        int16_t voltage_limit_raw = msg->data[kRxVoltageLimit];

        double angle_deg = static_cast<double>(angle_raw) * kTargetAngleScale;
        double velocity_rad_per_sec = static_cast<double>(velocity_raw) * kTargetVelocityScale;
        double target = static_cast<double>(target_raw) *
                        (mode == 1 ? kTargetAngleScale : kTargetVelocityScale);
        double voltage_limit = static_cast<double>(voltage_limit_raw) * kVoltageLimitScale;

        if (show_rx_logs_) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *this->get_clock(), 500,
                "RX mode=%d angle=%.2f deg velocity=%.2f rad/s (%.1f rpm) target=%.2f %s vlim=%.2f V",
                mode, angle_deg, velocity_rad_per_sec, rad_per_sec_to_rpm(velocity_rad_per_sec),
                target, mode == 1 ? "deg" : "rad/s", voltage_limit);
        }

        // 以降、受信データを使った処理を記述
        // 例:
        // if (mode == 0 && std::abs(velocity_rad_per_sec) > 50.0) {
        //     data_[kCmdEnable] = 0;
        // }

        // 受信データ処理ここまで
    }

    int tx_device_id_{};
    int rx_device_id_{};
    int publish_rate_ms_{};
    std::string joy_topic_;
    bool enable_on_start_{true};
    int start_mode_{};
    double start_velocity_rpm_{};
    double start_angle_deg_{};
    double start_voltage_limit_{12.0};
    bool show_rx_logs_{true};

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<int16_t> data_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    std::string figletout = "figlet ESC Ctrl Dev Frame";
    int result = std::system(figletout.c_str());
    if (result != 0) {
        std::cerr << "figlet is not installed. Continuing without banner." << std::endl;
    }

    rclcpp::spin(std::make_shared<HardWareControl>());
    rclcpp::shutdown();
    return 0;
}