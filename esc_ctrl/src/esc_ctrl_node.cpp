#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>

#include <sys/select.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

namespace {
    constexpr std::size_t kTx16Num = 24; // PC -> MCU

    constexpr float kTargetVelocityScale = 0.1f; // int16 <-> rad/s
    constexpr float kTargetAngleScale = 0.1f;    // int16 <-> deg
    constexpr float kVoltageLimitScale = 0.1f;   // int16 <-> V

    constexpr float kVelocityLimitScale = 0.1f; // int16 <-> rad/s
    constexpr float kCurrentLimitScale = 0.1f;  // int16 <-> A
    constexpr float kVelocityPidGainScale = 0.001f;
    constexpr float kAnglePGainScale = 0.01f;

    constexpr std::size_t kCmdEnable = 1;
    constexpr std::size_t kCmdMode = 2;
    constexpr std::size_t kCmdTargetVelocity = 3;
    constexpr std::size_t kCmdTargetAngle = 4;
    constexpr std::size_t kCmdVoltageLimit = 5;

    // runtime tuning (must match MCU config.hpp)
    constexpr std::size_t kCmdParamApplyMask = 6;
    constexpr std::size_t kCmdVelocityLimit = 7;
    constexpr std::size_t kCmdCurrentLimit = 8;
    constexpr std::size_t kCmdVelocityPidP = 9;
    constexpr std::size_t kCmdVelocityPidI = 10;
    constexpr std::size_t kCmdVelocityPidD = 11;
    constexpr std::size_t kCmdVelocityPidOutputRamp = 12;
    constexpr std::size_t kCmdVelocityLpfTfMs = 13;
    constexpr std::size_t kCmdAnglePGain = 14;

    constexpr int16_t kApplyVoltageLimit = (1 << 0);
    constexpr int16_t kApplyVelocityLimit = (1 << 1);
    constexpr int16_t kApplyCurrentLimit = (1 << 2);
    constexpr int16_t kApplyVelocityPid = (1 << 3);
    constexpr int16_t kApplyVelocityOutputRamp = (1 << 4);
    constexpr int16_t kApplyVelocityLpfTf = (1 << 5);
    constexpr int16_t kApplyAnglePGain = (1 << 6);

    constexpr int kParamApplyFrames = 10;

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
} // namespace

class EscCtrlNode : public rclcpp::Node {
public:
    EscCtrlNode() : Node("esc_ctrl") {
        device_id_ = this->declare_parameter<int>("device_id", 153);
        tx_period_ms_ = this->declare_parameter<int>("tx_period_ms", 20);
        log_period_ms_ = this->declare_parameter<int>("log_period_ms", 200);
        rx_force_log_period_ms_ = this->declare_parameter<int>("rx_force_log_period_ms", 5000);
        rx_change_epsilon_ = this->declare_parameter<double>("rx_change_epsilon", 0.5);
        rx_target_change_epsilon_ = this->declare_parameter<double>("rx_target_change_epsilon", 1.0);
        show_info_logs_ = this->declare_parameter<bool>("show_info_logs", false);
        fixed_enable_ = this->declare_parameter<int>("fixed_enable", 1);
        fixed_mode_ = this->declare_parameter<int>("fixed_mode", 0);
        fixed_target_ = this->declare_parameter<double>("fixed_target", 5.0);
        fixed_voltage_limit_ = this->declare_parameter<double>("fixed_voltage_limit", 12.0);

        // defaults that match the current MCU sketch (best-effort)
        velocity_limit_ = this->declare_parameter<double>("velocity_limit", 1500.0);
        current_limit_ = this->declare_parameter<double>("current_limit", 10.0);
        velocity_pid_p_ = this->declare_parameter<double>("velocity_pid_p", 0.02);
        velocity_pid_i_ = this->declare_parameter<double>("velocity_pid_i", 0.0);
        velocity_pid_d_ = this->declare_parameter<double>("velocity_pid_d", 0.0);
        velocity_pid_output_ramp_ = this->declare_parameter<double>("velocity_pid_output_ramp", 1000.0);
        velocity_lpf_tf_s_ = this->declare_parameter<double>("velocity_lpf_tf", 0.02);
        angle_p_gain_ = this->declare_parameter<double>("angle_p_gain", 8.0);

        fixed_mode_ = (fixed_mode_ == 1) ? 1 : 0;

        serial_tx_topic_ = "serial_tx_" + std::to_string(device_id_);
        serial_rx_topic_ = "serial_rx_" + std::to_string(device_id_);

        serial_rx_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            serial_rx_topic_, 10,
            std::bind(&EscCtrlNode::on_serial_rx, this, std::placeholders::_1));

        // Console/ROS commands
        cmd_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/esc_ctrl/cmd", 10,
            std::bind(&EscCtrlNode::on_cmd, this, std::placeholders::_1));
        params_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/esc_ctrl/cmd/params", 10,
            std::bind(&EscCtrlNode::on_params, this, std::placeholders::_1));

        serial_tx_pub_ =
            this->create_publisher<std_msgs::msg::Int16MultiArray>(serial_tx_topic_, 10);

        tx_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(std::max(1, tx_period_ms_)),
            std::bind(&EscCtrlNode::publish_command, this));

        if (show_info_logs_) {
            RCLCPP_INFO(this->get_logger(), "esc_ctrl started (keyboard command mode)");
            RCLCPP_INFO(this->get_logger(), "device_id=%d tx=%s rx=%s", device_id_,
                        serial_tx_topic_.c_str(), serial_rx_topic_.c_str());
            RCLCPP_INFO(this->get_logger(),
                        "Initial command: enable=%d mode=%d target=%.3f voltage_limit=%.3f", fixed_enable_,
                        fixed_mode_, fixed_target_, fixed_voltage_limit_);
            RCLCPP_INFO(this->get_logger(),
                        "Keyboard input: v<number> (velocity rad/s), p<number> (position deg). Example: v100, p90");
            RCLCPP_INFO(this->get_logger(),
                        "RX log mode: on-change (angle/vel/vlim eps=%.3f, target eps=%.3f), snapshot=%d ms",
                        rx_change_epsilon_, rx_target_change_epsilon_, rx_force_log_period_ms_);
        }

        stdin_is_tty_ = ::isatty(STDIN_FILENO) == 1;
        if (!stdin_is_tty_) {
            RCLCPP_WARN(this->get_logger(),
                        "stdin is not a TTY. Keyboard command input is disabled in this run."
                        " Use ros2 run for interactive control.");
        }

        last_rx_log_time_ = this->now();
    }

private:
    void on_cmd(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        const auto &d = msg->data;
        if (d.size() >= 1) {
            fixed_enable_ = (d[0] != 0.0f) ? 1 : 0;
        }
        if (d.size() >= 2) {
            fixed_mode_ = (d[1] >= 0.5f) ? 1 : 0;
        }
        if (d.size() >= 3) {
            fixed_target_ = d[2];
        }
        if (d.size() >= 4) {
            fixed_voltage_limit_ = d[3];
        }

        if (show_info_logs_) {
            RCLCPP_INFO(this->get_logger(), "CMD updated: enable=%d mode=%d target=%.3f vlim=%.2f",
                        fixed_enable_, fixed_mode_, fixed_target_, fixed_voltage_limit_);
        }
    }

    void on_params(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        const auto &d = msg->data;
        int16_t mask = 0;

        // order: [vlim, velocity_limit, current_limit, velP, velI, velD, ramp, lpf_tf_s, angleP]
        if (d.size() >= 1) {
            fixed_voltage_limit_ = d[0];
            mask |= kApplyVoltageLimit;
        }
        if (d.size() >= 2) {
            velocity_limit_ = d[1];
            mask |= kApplyVelocityLimit;
        }
        if (d.size() >= 3) {
            current_limit_ = d[2];
            mask |= kApplyCurrentLimit;
        }
        if (d.size() >= 6) {
            velocity_pid_p_ = d[3];
            velocity_pid_i_ = d[4];
            velocity_pid_d_ = d[5];
            mask |= kApplyVelocityPid;
        }
        if (d.size() >= 7) {
            velocity_pid_output_ramp_ = d[6];
            mask |= kApplyVelocityOutputRamp;
        }
        if (d.size() >= 8) {
            velocity_lpf_tf_s_ = d[7];
            mask |= kApplyVelocityLpfTf;
        }
        if (d.size() >= 9) {
            angle_p_gain_ = d[8];
            mask |= kApplyAnglePGain;
        }

        pending_param_apply_mask_ |= mask;
        if (mask != 0) {
            pending_param_apply_count_ = kParamApplyFrames;
        }

        if (show_info_logs_) {
            RCLCPP_INFO(this->get_logger(),
                        "PARAMS queued(mask=0x%04x): vlim=%.2f vel_lim=%.1f cur_lim=%.1f velPID[%.4f %.4f %.4f] ramp=%.1f lpf=%.3fs angP=%.2f",
                        (unsigned)pending_param_apply_mask_, fixed_voltage_limit_, velocity_limit_,
                        current_limit_, velocity_pid_p_, velocity_pid_i_, velocity_pid_d_,
                        velocity_pid_output_ramp_, velocity_lpf_tf_s_, angle_p_gain_);
        }
    }

    void poll_keyboard_command() {
        if (!stdin_is_tty_) {
            return;
        }

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);

        timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;

        const int ready = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);
        if (ready <= 0 || !FD_ISSET(STDIN_FILENO, &readfds)) {
            return;
        }

        std::string line;
        if (!std::getline(std::cin, line) || line.empty()) {
            return;
        }

        const char prefix = line[0];
        const std::string value_str = line.substr(1);

        try {
            const double value = std::stod(value_str);
            if (prefix == 'v' || prefix == 'V') {
                fixed_enable_ = 1;
                fixed_mode_ = 0;
                fixed_target_ = value;
            } else if (prefix == 'p' || prefix == 'P') {
                fixed_enable_ = 1;
                fixed_mode_ = 1;
                fixed_target_ = value;
            } else {
                RCLCPP_WARN(this->get_logger(),
                            "Unknown command: %s (use v<number> or p<number>)", line.c_str());
                return;
            }

            if (show_info_logs_) {
                RCLCPP_INFO(this->get_logger(), "Updated command: mode=%s target=%.3f %s",
                            (fixed_mode_ == 0) ? "velocity" : "angle",
                            fixed_target_,
                            (fixed_mode_ == 0) ? "rad/s" : "deg");
            }
        } catch (...) {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid input: %s (use v<number> or p<number>)", line.c_str());
        }
    }

    void publish_command() {
        poll_keyboard_command();

        std_msgs::msg::Int16MultiArray out;
        out.data.assign(kTx16Num, 0);

        out.data[kCmdEnable] = fixed_enable_ ? 1 : 0;
        out.data[kCmdMode] = static_cast<int16_t>(fixed_mode_);
        if (fixed_mode_ == 0) {
            out.data[kCmdTargetVelocity] =
                clamp_i16(std::lround(fixed_target_ / kTargetVelocityScale));
            out.data[kCmdTargetAngle] = 0;
        } else {
            out.data[kCmdTargetVelocity] = 0;
            out.data[kCmdTargetAngle] =
                clamp_i16(std::lround(fixed_target_ / kTargetAngleScale));
        }
        out.data[kCmdVoltageLimit] =
            clamp_i16(std::lround(fixed_voltage_limit_ / kVoltageLimitScale));

        // Always apply voltage_limit so operator feedback is immediate.
        // Other params are sent for a few frames after a request.
        int16_t queued_param_mask = 0;
        if (pending_param_apply_count_ > 0) {
            queued_param_mask = pending_param_apply_mask_;
            pending_param_apply_count_--;
            if (pending_param_apply_count_ == 0) {
                pending_param_apply_mask_ = 0;
            }
        }

        const int16_t apply_mask = static_cast<int16_t>(queued_param_mask | kApplyVoltageLimit);
        out.data[kCmdParamApplyMask] = apply_mask;

        out.data[kCmdVelocityLimit] = clamp_i16(std::lround(velocity_limit_ / kVelocityLimitScale));
        out.data[kCmdCurrentLimit] = clamp_i16(std::lround(current_limit_ / kCurrentLimitScale));
        out.data[kCmdVelocityPidP] = clamp_i16(std::lround(velocity_pid_p_ / kVelocityPidGainScale));
        out.data[kCmdVelocityPidI] = clamp_i16(std::lround(velocity_pid_i_ / kVelocityPidGainScale));
        out.data[kCmdVelocityPidD] = clamp_i16(std::lround(velocity_pid_d_ / kVelocityPidGainScale));
        out.data[kCmdVelocityPidOutputRamp] = clamp_i16(std::lround(velocity_pid_output_ramp_));
        out.data[kCmdVelocityLpfTfMs] = clamp_i16(std::lround(velocity_lpf_tf_s_ * 1000.0));
        out.data[kCmdAnglePGain] = clamp_i16(std::lround(angle_p_gain_ / kAnglePGainScale));

        serial_tx_pub_->publish(out);
    }

    void on_serial_rx(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        if (msg->data.size() < 6) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "serial_rx frame too short: %zu", msg->data.size());
            return;
        }

        const int mode = (msg->data[kRxMode] == 1) ? 1 : 0;
        const float angle = static_cast<float>(msg->data[kRxAngle]) * kTargetAngleScale;
        const float velocity = static_cast<float>(msg->data[kRxVelocity]) * kTargetVelocityScale;
        const float target = static_cast<float>(msg->data[kRxTarget]) *
                             (mode == 1 ? kTargetAngleScale : kTargetVelocityScale);
        const float vlim = static_cast<float>(msg->data[kRxVoltageLimit]) * kVoltageLimitScale;

        const bool changed =
            !has_last_rx_ ||
            std::fabs(angle - last_angle_) > rx_change_epsilon_ ||
            std::fabs(velocity - last_velocity_) > rx_change_epsilon_ ||
            std::fabs(target - last_target_) > rx_target_change_epsilon_ ||
            std::fabs(vlim - last_vlim_) > rx_change_epsilon_ ||
            mode != last_mode_;

        const auto now = this->now();
        const bool periodic_snapshot =
            (now - last_rx_log_time_).nanoseconds() / 1000000 >= std::max(200, rx_force_log_period_ms_);

        if (show_info_logs_ && (changed || periodic_snapshot)) {
            RCLCPP_INFO(this->get_logger(),
                        "RX angle=%.3f deg velocity=%.3f rad/s target=%.3f %s vlim=%.2fV",
                        angle, velocity, target,
                        mode == 1 ? "deg" : "rad/s", vlim);
            last_rx_log_time_ = now;
        }

        has_last_rx_ = true;
        last_angle_ = angle;
        last_velocity_ = velocity;
        last_target_ = target;
        last_vlim_ = vlim;
        last_mode_ = mode;
    }

    int device_id_;
    int tx_period_ms_;
    int log_period_ms_;
    int rx_force_log_period_ms_;
    double rx_change_epsilon_;
    double rx_target_change_epsilon_;
    bool show_info_logs_;
    int fixed_enable_;
    int fixed_mode_;
    double fixed_target_;
    double fixed_voltage_limit_;

    // runtime tuning params
    double velocity_limit_{0.0};
    double current_limit_{0.0};
    double velocity_pid_p_{0.0};
    double velocity_pid_i_{0.0};
    double velocity_pid_d_{0.0};
    double velocity_pid_output_ramp_{0.0};
    double velocity_lpf_tf_s_{0.0};
    double angle_p_gain_{0.0};

    int16_t pending_param_apply_mask_{0};
    int pending_param_apply_count_{0};

    std::string serial_tx_topic_;
    std::string serial_rx_topic_;

    bool has_last_rx_{false};
    float last_angle_{0.0f};
    float last_velocity_{0.0f};
    float last_target_{0.0f};
    float last_vlim_{0.0f};
    int last_mode_{0};
    rclcpp::Time last_rx_log_time_;
    bool stdin_is_tty_{false};

    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr serial_rx_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr serial_tx_pub_;
    rclcpp::TimerBase::SharedPtr tx_timer_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr params_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EscCtrlNode>());
    rclcpp::shutdown();
    return 0;
}
