// RobStride EduLite 05 の回転数などをターミナルに1行で表示し続けるノード(指令は一切送らない)。
// robstride/motor<N>/state と robstride/motor<N>/cmd を購読し、print_rate_hzで上書き表示する。
// (damiao_ctrlのmotor_monitorと同じ作り。温度と故障の表示を追加)
//
// 表示: モード | 指令 | 実速度[rpm](rad/s) | 最高[rpm] | トルク | 位置 | 温度 | 帰還
//   - 速度・位置は出力軸。位置は多回転(1回転=6.283rad)。
//   - 故障・ESP32による停止・帰還途絶の時は、その行を残して改行する。

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "robstride_ctrl/slot_layout.hpp"

using namespace std::chrono_literals;

namespace robstride_ctrl
{

constexpr double kRadPerSecToRpm = 60.0 / (2.0 * M_PI);

class MotorMonitorNode : public rclcpp::Node
{
public:
  MotorMonitorNode()
  : Node("motor_monitor")
  {
    const int motor_number = declare_parameter<int>("motor_number", 1);
    const double print_rate_hz = declare_parameter<double>("print_rate_hz", 10.0);
    if (print_rate_hz <= 0.0) {
      throw std::runtime_error("print_rate_hzは正の値を指定してください");
    }

    state_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "robstride/motor" + std::to_string(motor_number) + "/state", 10,
      std::bind(&MotorMonitorNode::on_state, this, std::placeholders::_1));
    cmd_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "robstride/motor" + std::to_string(motor_number) + "/cmd", 10,
      std::bind(&MotorMonitorNode::on_cmd, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / print_rate_hz),
      std::bind(&MotorMonitorNode::print_line, this));

    std::printf(
      "motor_monitor: robstride/motor%d を表示します(Ctrl+Cで終了。指令は送りません)\n",
      motor_number);
    std::fflush(stdout);
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  void on_state(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 7) {
      return;
    }
    position_ = msg->data[0];
    velocity_ = msg->data[1];
    torque_ = msg->data[2];
    motor_mode_ = static_cast<int16_t>(msg->data[3]);
    feedback_ok_ = msg->data[4] > 0.5f;
    flags_ = static_cast<uint16_t>(msg->data[5]);
    temperature_ = msg->data[6];
    last_state_time_ = SteadyClock::now();
    have_state_ = true;
    peak_rpm_ = std::max(peak_rpm_, std::abs(velocity_) * kRadPerSecToRpm);
  }

  void on_cmd(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) {
      return;
    }
    cmd_mode_ = msg->data[0];
    cmd_target_ = msg->data[1];
    have_cmd_ = true;
  }

  void print_line()
  {
    const bool state_alive = have_state_ &&
      std::chrono::duration<double>(SteadyClock::now() - last_state_time_).count() < 0.5;
    if (!state_alive) {
      print_persistent(
        "stateが届いていません(robstride_driver_node・ros2canの起動と、"
        "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOSTを確認)");
      return;
    }

    std::string cmd_text = "指令 なし         ";
    if (have_cmd_) {
      char buf[64];
      if (cmd_mode_ <= -0.5) {
        std::snprintf(buf, sizeof(buf), "指令 停止         ");
      } else if (cmd_mode_ < 0.5) {
        std::snprintf(buf, sizeof(buf), "指令 %+7.1frpm   ", cmd_target_ * kRadPerSecToRpm);
      } else {
        std::snprintf(buf, sizeof(buf), "指令 位置%+7.2frad", cmd_target_);
      }
      cmd_text = buf;
    }

    std::string status;
    if (!feedback_ok_) {
      status = "途絶";
    } else if (flags_ & kFlagLatched) {
      status = "ESP32が停止中(DISABLEを送ると解除)";
    } else {
      status = "OK";
    }
    for (const char * name : fault_names(flags_ & kFaultMask)) {
      status += std::string(" 故障:") + name;
    }

    char line[320];
    std::snprintf(
      line, sizeof(line),
      "%-6s | %s | 実速度 %+7.1frpm (%+6.2frad/s) | 最高 %6.1frpm | トルク %+5.2fN·m | "
      "位置 %+8.3frad | %4.1f℃ | %s",
      motor_mode_name(motor_mode_), cmd_text.c_str(), velocity_ * kRadPerSecToRpm, velocity_,
      peak_rpm_, torque_, position_, temperature_, status.c_str());

    if (status != "OK") {
      print_persistent(line);
      return;
    }
    last_persistent_.clear();
    std::printf("\r%s\033[K", line);  // 行頭に戻って上書きし、行末の残りを消す
    std::fflush(stdout);
  }

  // 異常の行は上書きせずに残す(同じ内容は繰り返さない)
  void print_persistent(const std::string & text)
  {
    if (text == last_persistent_) {
      return;
    }
    std::printf("\r%s\033[K\n", text.c_str());
    std::fflush(stdout);
    last_persistent_ = text;
  }

  double position_{0.0};
  double velocity_{0.0};
  double torque_{0.0};
  double temperature_{0.0};
  int16_t motor_mode_{kMotorModeReset};
  uint16_t flags_{0};
  bool feedback_ok_{false};
  bool have_state_{false};
  SteadyClock::time_point last_state_time_{};
  double peak_rpm_{0.0};

  double cmd_mode_{-1.0};
  double cmd_target_{0.0};
  bool have_cmd_{false};

  std::string last_persistent_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robstride_ctrl

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robstride_ctrl::MotorMonitorNode>();
  rclcpp::spin(node);
  std::printf("\n");
  rclcpp::shutdown();
  return 0;
}
