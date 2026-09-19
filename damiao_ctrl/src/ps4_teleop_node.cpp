// PS4コントローラ(USB有線, joy_linux/joyパッケージ経由)でDamiaoモータの
// 速度制御(VELモード)をテレオペするノード。
//
// 安全設計: R1(既定)を押している間だけ指令を有効化する「デッドマンスイッチ」
// 方式。R1を離す、またはOptionsボタン(既定)を押すと即座にDISABLE(E-STOP)
// 指令を送る。ボタン割り当ては実機のジョイスティックドライバによって
// ズレることがあるため、実際に使う前に `ros2 topic echo /joy` で
// axes/buttonsのインデックスを必ず確認すること(README参照)。

#include <algorithm>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

namespace damiao_ctrl
{

class Ps4TeleopNode : public rclcpp::Node
{
public:
  Ps4TeleopNode()
  : Node("ps4_teleop_node")
  {
    axis_index_ = declare_parameter<int>("velocity_axis_index", 1);       // 左スティック上下(既定)
    deadman_button_index_ = declare_parameter<int>("deadman_button_index", 5);  // R1(既定)
    estop_button_index_ = declare_parameter<int>("estop_button_index", 9);      // Options(既定)
    max_vel_radps_ = declare_parameter<double>("max_vel_radps", 3.0);     // 初回動作確認用に控えめな値
    deadzone_ = declare_parameter<double>("deadzone", 0.08);
    motor_number_ = declare_parameter<int>("motor_number", 1);

    cmd_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "damiao/motor" + std::to_string(motor_number_) + "/cmd", 10);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Ps4TeleopNode::on_joy, this, std::placeholders::_1));

    // Joyメッセージが途絶えた場合に備え、一定周期で最新状態を再送する。
    // (コントローラのUSB切断等でこのノード自体が指令を送れなくなった場合
    //  最終的な安全確保はdamiao_driver_node/bridge_node側のon_shutdown・
    //  タイムアウト処理に委ねる)
    publish_timer_ = create_wall_timer(20ms, std::bind(&Ps4TeleopNode::publish_cmd, this));

    RCLCPP_INFO(
      get_logger(),
      "ps4_teleop_node: axis=%d deadman_button=%d estop_button=%d max_vel=%.2frad/s "
      "(実際のボタン配置は `ros2 topic echo /joy` で要確認)",
      axis_index_, deadman_button_index_, estop_button_index_, max_vel_radps_);
  }

private:
  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const bool axis_ok = axis_index_ >= 0 && static_cast<size_t>(axis_index_) < msg->axes.size();
    const bool deadman_ok =
      deadman_button_index_ >= 0 && static_cast<size_t>(deadman_button_index_) < msg->buttons.size();
    const bool estop_ok =
      estop_button_index_ >= 0 && static_cast<size_t>(estop_button_index_) < msg->buttons.size();

    const bool estop_pressed = estop_ok && msg->buttons[estop_button_index_] != 0;
    const bool deadman_held = deadman_ok && msg->buttons[deadman_button_index_] != 0;

    if (estop_pressed || !deadman_held || !axis_ok) {
      enabled_ = false;
      target_vel_ = 0.0;
      return;
    }

    double raw = static_cast<double>(msg->axes[axis_index_]);
    if (std::abs(raw) < deadzone_) {
      raw = 0.0;
    }
    enabled_ = true;
    target_vel_ = raw * max_vel_radps_;
  }

  void publish_cmd()
  {
    std_msgs::msg::Float32MultiArray msg;
    if (enabled_) {
      // control_mode=0.0(VEL), target=target_vel_[rad/s], vel_limit未使用
      msg.data = {0.0f, static_cast<float>(target_vel_), 0.0f};
    } else {
      // control_mode=-1.0(DISABLE)
      msg.data = {-1.0f, 0.0f, 0.0f};
    }
    cmd_pub_->publish(msg);
  }

  int axis_index_{1};
  int deadman_button_index_{5};
  int estop_button_index_{9};
  double max_vel_radps_{3.0};
  double deadzone_{0.08};
  int motor_number_{1};

  bool enabled_{false};
  double target_vel_{0.0};

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace damiao_ctrl

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<damiao_ctrl::Ps4TeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
