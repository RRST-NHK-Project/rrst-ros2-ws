// Damiaoモータの回転数などをターミナルに表示し続けるノード(指令は一切送らない)。
// motor_numbers(既定[1]、2台なら[1, 2])の各モータについて、damiao/motor<N>/state と
// damiao/motor<N>/cmd を購読し、1台1行でprint_rate_hzごとに上書き表示する。
//
// 表示: [モータN] 状態 | 指令速度[rpm] | 実速度[rpm](rad/s) | 最高[rpm] | トルク | 位置 | 帰還
//   - 速度は出力軸。モータの速度の分解能は約0.098rad/s(約0.94rpm)で0を表せないため、
//     止まっていても±0.5rpm程度を示す。
//   - 故障(状態が0/1以外)・帰還途絶・stateが届かない、になった時と、そこから戻った時は、
//     表示の上にその出来事を1行残す(上書きされない)。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "damiao_ctrl/slot_layout.hpp"

using namespace std::chrono_literals;

namespace damiao_ctrl
{

constexpr double kRadPerSecToRpm = 60.0 / (2.0 * M_PI);

class MotorMonitorNode : public rclcpp::Node
{
public:
  MotorMonitorNode()
  : Node("motor_monitor")
  {
    const auto motor_numbers =
      declare_parameter<std::vector<int64_t>>("motor_numbers", std::vector<int64_t>{1});
    const double print_rate_hz = declare_parameter<double>("print_rate_hz", 10.0);
    if (print_rate_hz <= 0.0 || motor_numbers.empty()) {
      throw std::runtime_error("print_rate_hzは正の値、motor_numbersは1つ以上にしてください");
    }

    std::string names;
    for (const auto number : motor_numbers) {
      auto m = std::make_unique<Motor>();
      m->number = static_cast<int>(number);
      const std::string prefix = "damiao/motor" + std::to_string(m->number);
      Motor * raw = m.get();
      m->state_sub = create_subscription<std_msgs::msg::Float32MultiArray>(
        prefix + "/state", 10,
        [raw](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {on_state(*raw, msg);});
      m->cmd_sub = create_subscription<std_msgs::msg::Float32MultiArray>(
        prefix + "/cmd", 10,
        [raw](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {on_cmd(*raw, msg);});
      names += " " + prefix;
      motors_.push_back(std::move(m));
    }
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / print_rate_hz),
      std::bind(&MotorMonitorNode::print_block, this));

    std::printf("motor_monitor:%s を表示します(Ctrl+Cで終了。指令は送りません)\n", names.c_str());
    std::fflush(stdout);
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  struct Motor
  {
    int number{1};
    double position{0.0};
    double velocity{0.0};
    double torque{0.0};
    double internal_pos{std::nan("")};
    int16_t motor_state{kMotorStateDisabled};
    bool feedback_ok{false};
    bool have_state{false};
    SteadyClock::time_point last_state_time{};
    double peak_rpm{0.0};
    double cmd_mode{-1.0};
    double cmd_target{0.0};
    bool have_cmd{false};
    std::string last_condition;  // 前回の「異常/正常」の区分(出来事の行を出すため)
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr state_sub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub;
  };

  static void on_state(Motor & m, const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 5) {
      return;
    }
    m.position = msg->data[0];
    m.velocity = msg->data[1];
    m.torque = msg->data[2];
    m.internal_pos = msg->data.size() >= 6 ? msg->data[5] : std::nan("");
    m.motor_state = static_cast<int16_t>(msg->data[3]);
    m.feedback_ok = msg->data[4] > 0.5f;
    m.last_state_time = SteadyClock::now();
    m.have_state = true;
    m.peak_rpm = std::max(m.peak_rpm, std::abs(m.velocity) * kRadPerSecToRpm);
  }

  static void on_cmd(Motor & m, const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) {
      return;
    }
    m.cmd_mode = msg->data[0];
    m.cmd_target = msg->data[1];
    m.have_cmd = true;
  }

  // 1台分の表示行と、異常/正常の区分を作る
  static std::string make_line(const Motor & m, std::string & condition)
  {
    char tag[32];
    std::snprintf(tag, sizeof(tag), "[モータ%d] ", m.number);
    const bool state_alive = m.have_state &&
      std::chrono::duration<double>(SteadyClock::now() - m.last_state_time).count() < 0.5;
    if (!state_alive) {
      condition = "stateなし";
      return std::string(tag) + "stateが届いていません(damiao_driver_node・ros2canの起動と、"
             "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOSTを確認)";
    }
    char cmd_text[64];
    if (!m.have_cmd) {
      std::snprintf(cmd_text, sizeof(cmd_text), "指令 なし        ");
    } else if (m.cmd_mode <= -0.5) {
      std::snprintf(cmd_text, sizeof(cmd_text), "指令 停止        ");
    } else if (m.cmd_mode < 0.5) {
      std::snprintf(cmd_text, sizeof(cmd_text), "指令 %+7.1frpm  ", m.cmd_target * kRadPerSecToRpm);
    } else {
      std::snprintf(cmd_text, sizeof(cmd_text), "指令 角度%+6.2frad", m.cmd_target);
    }
    char internal_text[32];
    if (std::isnan(m.internal_pos)) {
      std::snprintf(internal_text, sizeof(internal_text), "不明");
    } else {
      std::snprintf(internal_text, sizeof(internal_text), "%+.1frad", m.internal_pos);
    }
    char line[320];
    std::snprintf(
      line, sizeof(line),
      "%s%-8s | %s | 実速度 %+7.1frpm (%+6.2frad/s) | 最高 %6.1frpm | トルク %+5.2fN·m | "
      "位置 %+6.2frad | 内部 %s | 帰還 %s",
      tag, motor_state_name(m.motor_state), cmd_text, m.velocity * kRadPerSecToRpm, m.velocity,
      m.peak_rpm, m.torque, m.position, internal_text, m.feedback_ok ? "OK" : "途絶");
    if (!m.feedback_ok) {
      condition = "帰還途絶";
    } else if (!std::isnan(m.internal_pos) && std::abs(m.internal_pos) > kPosVelMaxInternalRad) {
      condition = "内部の位置が大きい(角度制御不可。電源を入れ直すと0に戻る)";
    } else if (is_motor_fault(m.motor_state)) {
      condition = std::string("故障: ") + motor_state_name(m.motor_state);
    } else {
      condition = "正常";
    }
    return line;
  }

  void print_block()
  {
    std::vector<std::string> lines;
    std::vector<std::string> events;
    for (auto & m : motors_) {
      std::string condition;
      lines.push_back(make_line(*m, condition));
      if (condition != m->last_condition) {
        if (condition != "正常" || !m->last_condition.empty()) {
          // 異常になった時は表示行そのもの、戻った時は「正常に戻った」を残す
          events.push_back(condition == "正常" ?
            "[モータ" + std::to_string(m->number) + "] 正常に戻りました" : lines.back());
        }
        m->last_condition = condition;
      }
    }
    if (printed_) {
      std::printf("\033[%zuA", lines.size());  // 前回の表示の先頭行へ戻る
    }
    for (const auto & e : events) {
      std::printf("\r%s\033[K\n", e.c_str());  // 出来事の行(この後の表示で上書きされない)
    }
    for (const auto & l : lines) {
      std::printf("\r%s\033[K\n", l.c_str());
    }
    std::fflush(stdout);
    printed_ = true;
  }

  std::vector<std::unique_ptr<Motor>> motors_;
  bool printed_{false};
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace damiao_ctrl

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<damiao_ctrl::MotorMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
