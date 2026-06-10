#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "common/PID.hpp"
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

namespace {
    constexpr std::size_t kTx16Num = 24; // PC -> MCU
    constexpr double kOneTurnDeg = 360.0;
    constexpr double kWrapDetectThresholdDeg = 180.0;

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

class EscPositionPidNode : public rclcpp::Node {
public:
    EscPositionPidNode()
        : Node("esc_position_pid") {
        device_id_ = this->declare_parameter<int>("device_id", 153);
        rx_device_id_ = this->declare_parameter<int>("rx_device_id", device_id_);
        tx_period_ms_ = this->declare_parameter<int>("tx_period_ms", 20);
        command_topic_prefix_ = this->declare_parameter<std::string>("command_topic_prefix", "esc_cmd_pos");
        command_rpm_scale_ = this->declare_parameter<double>("command_rpm_scale", 1.0);
        voltage_limit_ = this->declare_parameter<double>("voltage_limit", 12.0);
        position_tolerance_deg_ = this->declare_parameter<double>("position_tolerance_deg", 1.0);
        enable_on_command_ = this->declare_parameter<bool>("enable_on_command", true);
        show_info_logs_ = this->declare_parameter<bool>("show_info_logs", true);

        const double pid_kp = this->declare_parameter<double>("pid_kp", 10.0);
        const double pid_ki = this->declare_parameter<double>("pid_ki", 0.0);
        const double pid_kd = this->declare_parameter<double>("pid_kd", 0.0);
        pid_max_rpm_ = this->declare_parameter<double>("pid_max_rpm", 3000.0);

        pid_controller_ = std::make_unique<PIDController>(
            static_cast<float>(pid_kp),
            static_cast<float>(pid_ki),
            static_cast<float>(pid_kd),
            static_cast<float>(pid_max_rpm_));

        command_topic_ = command_topic_prefix_ + "_" + std::to_string(device_id_);
        serial_tx_topic_ = "serial_tx_" + std::to_string(device_id_);
        serial_rx_topic_ = "serial_rx_" + std::to_string(rx_device_id_);

        command_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            command_topic_, 10,
            std::bind(&EscPositionPidNode::on_command, this, std::placeholders::_1));

        serial_rx_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            serial_rx_topic_, 10,
            std::bind(&EscPositionPidNode::on_serial_rx, this, std::placeholders::_1));

        serial_tx_pub_ =
            this->create_publisher<std_msgs::msg::Int16MultiArray>(serial_tx_topic_, 10);

        tx_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(std::max(1, tx_period_ms_)),
            std::bind(&EscPositionPidNode::publish_command, this));

        tx_data_.assign(kTx16Num, 0);
        last_control_time_ = this->now();

        if (show_info_logs_) {
            RCLCPP_INFO(this->get_logger(), "esc_position_pid started");
            RCLCPP_INFO(this->get_logger(), "topics: cmd=%s tx=%s rx=%s", command_topic_.c_str(),
                        serial_tx_topic_.c_str(), serial_rx_topic_.c_str());
            RCLCPP_INFO(this->get_logger(),
                        "Command format: Float64MultiArray [target_deg, max_rpm]");
            RCLCPP_INFO(this->get_logger(),
                        "PID gains: kp=%.3f ki=%.3f kd=%.3f max_rpm=%.1f", pid_kp, pid_ki, pid_kd,
                        pid_max_rpm_);
        }
    }

private:
    void on_command(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "command frame too short: %zu", msg->data.size());
            return;
        }

        target_angle_deg_ = msg->data[0];
        target_limit_rpm_ = std::fabs(msg->data[1] * command_rpm_scale_);

        has_command_ = true;
        pid_controller_->set_target(static_cast<float>(target_angle_deg_));
        pid_controller_->reset();
        last_control_time_ = this->now();

        if (show_info_logs_) {
            RCLCPP_INFO(this->get_logger(),
                        "New command: topic=%s target=%.3f deg limit=%.3f rpm",
                        command_topic_.c_str(), target_angle_deg_, target_limit_rpm_);
        }
    }

    void on_serial_rx(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        if (msg->data.size() <= kRxVoltageLimit) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "serial_rx frame too short: %zu", msg->data.size());
            return;
        }

        const double wrapped_angle_deg = static_cast<double>(msg->data[kRxAngle]) * kTargetAngleScale;
        if (!has_angle_reference_) {
            current_angle_deg_ = wrapped_angle_deg;
            last_wrapped_angle_deg_ = wrapped_angle_deg;
            has_angle_reference_ = true;
        } else {
            double delta_deg = wrapped_angle_deg - last_wrapped_angle_deg_;
            if (delta_deg > kWrapDetectThresholdDeg) {
                delta_deg -= kOneTurnDeg;
            } else if (delta_deg < -kWrapDetectThresholdDeg) {
                delta_deg += kOneTurnDeg;
            }
            current_angle_deg_ += delta_deg;
            last_wrapped_angle_deg_ = wrapped_angle_deg;
        }

        wrapped_angle_deg_ = wrapped_angle_deg;
        current_velocity_rad_per_sec_ =
            static_cast<double>(msg->data[kRxVelocity]) * kTargetVelocityScale;
        current_mode_ = (msg->data[kRxMode] == 1) ? 1 : 0;
        current_voltage_limit_ =
            static_cast<double>(msg->data[kRxVoltageLimit]) * kVoltageLimitScale;
        has_feedback_ = true;
    }

    void publish_command() {
        std_msgs::msg::Int16MultiArray out;
        out.data.assign(kTx16Num, 0);

        out.data[kCmdMode] = 0;
        out.data[kCmdVoltageLimit] =
            clamp_i16(std::lround(voltage_limit_ / kVoltageLimitScale));

        if (has_command_ && has_feedback_ && enable_on_command_) {
            const auto now = this->now();
            double dt = (now - last_control_time_).nanoseconds() * 1.0e-9;
            dt = std::max(dt, 1.0e-4);
            last_control_time_ = now;

            pid_controller_->set_target(static_cast<float>(target_angle_deg_));
            double commanded_rpm = pid_controller_->update(static_cast<float>(current_angle_deg_),
                                                           static_cast<float>(dt));
            commanded_rpm = std::clamp(commanded_rpm, -pid_max_rpm_, pid_max_rpm_);
            commanded_rpm = std::clamp(commanded_rpm, -target_limit_rpm_, target_limit_rpm_);

            const double position_error = target_angle_deg_ - current_angle_deg_;
            if (std::fabs(position_error) <= position_tolerance_deg_) {
                commanded_rpm = 0.0;
            }

            out.data[kCmdEnable] = 1;
            out.data[kCmdTargetVelocity] =
                clamp_i16(std::lround(rpm_to_rad_per_sec(commanded_rpm) / kTargetVelocityScale));
            out.data[kCmdTargetAngle] = 0;

            if (show_info_logs_) {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 500,
                    "PID topic=%s target=%.2f deg current=%.2f deg wrapped=%.2f deg err=%.2f deg cmd=%.2f rpm rx_vel=%.2f rpm rx_mode=%d rx_vlim=%.2fV",
                    command_topic_.c_str(), target_angle_deg_, current_angle_deg_, wrapped_angle_deg_, position_error,
                    commanded_rpm, rad_per_sec_to_rpm(current_velocity_rad_per_sec_),
                    current_mode_, current_voltage_limit_);
            }
        }

        tx_data_ = out.data;
        serial_tx_pub_->publish(out);
    }

    int device_id_{};
    int rx_device_id_{};
    int tx_period_ms_{};
    std::string command_topic_;
    std::string command_topic_prefix_;
    std::string serial_tx_topic_;
    std::string serial_rx_topic_;
    double command_rpm_scale_{1.0};
    double voltage_limit_{12.0};
    double position_tolerance_deg_{1.0};
    double pid_max_rpm_{3000.0};
    bool enable_on_command_{true};
    bool show_info_logs_{true};

    bool has_command_{false};
    bool has_feedback_{false};
    bool has_angle_reference_{false};
    double target_angle_deg_{0.0};
    double target_limit_rpm_{0.0};
    double current_angle_deg_{0.0};
    double wrapped_angle_deg_{0.0};
    double last_wrapped_angle_deg_{0.0};
    double current_velocity_rad_per_sec_{0.0};
    int current_mode_{0};
    double current_voltage_limit_{0.0};
    rclcpp::Time last_control_time_;

    std::unique_ptr<PIDController> pid_controller_;
    std::vector<int16_t> tx_data_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr serial_rx_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr serial_tx_pub_;
    rclcpp::TimerBase::SharedPtr tx_timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EscPositionPidNode>());
    rclcpp::shutdown();
    return 0;
}