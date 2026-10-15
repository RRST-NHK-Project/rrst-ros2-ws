// PS4コントローラ(/joy)で CubeMars AK40-10 を速度/角度操作するteleopノード。
// cubemars_driver_node へ cubemars/cmd_velocity・cmd_angle・stop を送るだけで、
// スロット変換や最終的な安全リミットはドライバ側が受け持つ。
//
// 操作(既定のボタン番号はチームのsoki_sim/joy_teleop_nodeと同じjoy_nodeの割り当て):
//   R1 (押している間だけ有効) : デッドマン。離すと停止指令を送る。
//   □ : 速度モード <-> 角度モード の切り替え。切り替えたら必ず一度停止し、
//       R1を押し直すまで動かない(モード切替の瞬間に動き出さないようにするため)。
//   速度モード: 左スティック上下 で ±max_rpm
//   角度モード: △ で +angle_step_deg、× で -angle_step_deg、○ で 0deg、
//              左スティック上下 で jog_speed_dps の速さで連続的に動かす
//   R1を押した時点の現在角度を角度モードの基準にする(位置ジャンプ防止)。
//
// /joy が joy_timeout_sec 以上届かない(コントローラ切断・joy_node停止)場合も停止する。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"

namespace cubemars_ctrl
{

class Ps4TeleopNode : public rclcpp::Node
{
public:
  Ps4TeleopNode()
  : Node("ps4_teleop_node")
  {
    deadman_button_ = declare_parameter<int>("deadman_button", 5);       // R1
    mode_toggle_button_ = declare_parameter<int>("mode_toggle_button", 3);  // □
    angle_plus_button_ = declare_parameter<int>("angle_plus_button", 2);    // △
    angle_minus_button_ = declare_parameter<int>("angle_minus_button", 0);  // ×
    angle_zero_button_ = declare_parameter<int>("angle_zero_button", 1);    // ○
    stick_axis_ = declare_parameter<int>("stick_axis", 1);                  // 左スティック上下
    invert_stick_ = declare_parameter<bool>("invert_stick", false);
    deadzone_ = declare_parameter<double>("deadzone", 0.1);
    max_rpm_ = declare_parameter<double>("max_rpm", 60.0);
    angle_step_deg_ = declare_parameter<double>("angle_step_deg", 90.0);
    jog_speed_dps_ = declare_parameter<double>("jog_speed_dps", 90.0);
    angle_limit_deg_ = declare_parameter<double>("angle_limit_deg", 360.0);
    joy_timeout_sec_ = declare_parameter<double>("joy_timeout_sec", 0.5);
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 50.0);

    vel_pub_ = create_publisher<std_msgs::msg::Float64>("cubemars/cmd_velocity", 10);
    angle_pub_ = create_publisher<std_msgs::msg::Float64>("cubemars/cmd_angle", 10);
    stop_pub_ = create_publisher<std_msgs::msg::Empty>("cubemars/stop", 10);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Ps4TeleopNode::on_joy, this, std::placeholders::_1));
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "cubemars/joint_state", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!msg->position.empty()) {
          measured_deg_ = msg->position[0] * 180.0 / M_PI;
        }
      });

    dt_ = 1.0 / publish_rate_hz;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_), std::bind(&Ps4TeleopNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "ps4_teleop_node 起動: R1(button %d)を押している間だけ有効 / □(button %d)でモード切替 / "
      "速度モード最大%.0frpm / 角度モード△×で±%.0fdeg, ○で0deg",
      deadman_button_, mode_toggle_button_, max_rpm_, angle_step_deg_);
    RCLCPP_INFO(get_logger(), "現在のモード: 速度");
  }

private:
  bool button(const sensor_msgs::msg::Joy & joy, int index) const
  {
    return index >= 0 && index < static_cast<int>(joy.buttons.size()) && joy.buttons[index] != 0;
  }

  double stick(const sensor_msgs::msg::Joy & joy) const
  {
    if (stick_axis_ < 0 || stick_axis_ >= static_cast<int>(joy.axes.size())) {
      return 0.0;
    }
    double v = joy.axes[stick_axis_];
    if (invert_stick_) {v = -v;}
    if (std::abs(v) < deadzone_) {return 0.0;}
    // デッドゾーンの外側を0〜1に引き伸ばす(デッドゾーン境界で値が飛ばないように)
    const double sign = v > 0.0 ? 1.0 : -1.0;
    return sign * std::clamp((std::abs(v) - deadzone_) / (1.0 - deadzone_), 0.0, 1.0);
  }

  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_time_ = now();
    joy_ = *msg;

    const bool toggle = button(*msg, mode_toggle_button_);
    if (toggle && !last_toggle_) {
      angle_mode_ = !angle_mode_;
      send_stop();
      wait_deadman_release_ = true;
      angle_target_.reset();
      RCLCPP_INFO(
        get_logger(), "モードを %s に切り替えました。R1を押し直すと動きます。",
        angle_mode_ ? "角度" : "速度");
    }
    last_toggle_ = toggle;

    // 角度モードのボタン操作(押した瞬間に1回だけ)
    const bool plus = button(*msg, angle_plus_button_);
    const bool minus = button(*msg, angle_minus_button_);
    const bool zero = button(*msg, angle_zero_button_);
    if (angle_mode_ && angle_target_) {
      if (plus && !last_plus_) {*angle_target_ += angle_step_deg_;}
      if (minus && !last_minus_) {*angle_target_ -= angle_step_deg_;}
      if (zero && !last_zero_) {*angle_target_ = 0.0;}
      *angle_target_ = std::clamp(*angle_target_, -angle_limit_deg_, angle_limit_deg_);
    }
    last_plus_ = plus;
    last_minus_ = minus;
    last_zero_ = zero;
  }

  void on_timer()
  {
    const bool joy_alive = joy_ &&
      (now() - last_joy_time_).seconds() < joy_timeout_sec_;
    const bool deadman = joy_alive && button(*joy_, deadman_button_);

    if (!deadman) {
      if (active_) {
        send_stop();
        RCLCPP_INFO(
          get_logger(), joy_alive ? "R1が離されたため停止しました。" :
          "/joy が途絶えたため停止しました(コントローラの接続を確認)。");
      }
      active_ = false;
      wait_deadman_release_ = false;
      angle_target_.reset();
      return;
    }
    if (wait_deadman_release_) {
      return;  // モード切替後はR1を押し直すまで何も送らない
    }

    if (!active_) {
      RCLCPP_INFO(get_logger(), "有効(%sモード)", angle_mode_ ? "角度" : "速度");
    }
    active_ = true;

    if (!angle_mode_) {
      std_msgs::msg::Float64 msg;
      msg.data = stick(*joy_) * max_rpm_;
      vel_pub_->publish(msg);
      return;
    }

    if (!angle_target_) {
      if (!measured_deg_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "cubemars/joint_state が来ていないため角度モードを開始できません。");
        return;
      }
      angle_target_ = std::clamp(*measured_deg_, -angle_limit_deg_, angle_limit_deg_);
    }
    *angle_target_ += stick(*joy_) * jog_speed_dps_ * dt_;
    *angle_target_ = std::clamp(*angle_target_, -angle_limit_deg_, angle_limit_deg_);

    std_msgs::msg::Float64 msg;
    msg.data = *angle_target_;
    angle_pub_->publish(msg);
  }

  void send_stop()
  {
    stop_pub_->publish(std_msgs::msg::Empty());
  }

  int deadman_button_{5};
  int mode_toggle_button_{3};
  int angle_plus_button_{2};
  int angle_minus_button_{0};
  int angle_zero_button_{1};
  int stick_axis_{1};
  bool invert_stick_{false};
  double deadzone_{0.1};
  double max_rpm_{60.0};
  double angle_step_deg_{90.0};
  double jog_speed_dps_{90.0};
  double angle_limit_deg_{360.0};
  double joy_timeout_sec_{0.5};
  double dt_{0.02};

  std::optional<sensor_msgs::msg::Joy> joy_;
  rclcpp::Time last_joy_time_{0, 0, RCL_ROS_TIME};
  std::optional<double> measured_deg_;
  std::optional<double> angle_target_;
  bool angle_mode_{false};
  bool active_{false};
  bool wait_deadman_release_{false};
  bool last_toggle_{false};
  bool last_plus_{false};
  bool last_minus_{false};
  bool last_zero_{false};

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cubemars_ctrl

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cubemars_ctrl::Ps4TeleopNode>());
  rclcpp::shutdown();
  return 0;
}
