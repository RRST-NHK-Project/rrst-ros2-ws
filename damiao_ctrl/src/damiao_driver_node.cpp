// 1台のDamiaoモータ(DM2325+DM3520)向け高レベル駆動ノード。
// damiao/motor<N>/cmd を購読してVEL/POS_VELの指令をserial_tx_<device_id>の
// 該当6スロットへ書き込み、serial_rx_<device_id>から自分のモータ分の
// フィードバックを damiao/motor<N>/state として配信する。
//
// cmd (std_msgs/Float32MultiArray, data=[control_mode, target, vel_limit]):
//   control_mode: -1.0=DISABLE(E-STOP), 0.0=VELモード(targetはrad/s),
//                 1.0=POS_VELモード(targetはrad, vel_limitで移動速度上限をrad/sで指定)
// state (std_msgs/Float32MultiArray, data=[position_rad, velocity_rad_s, torque_Nm])
//
// 同一bridge_nodeに対して駆動ノードは1つだけを想定している(複数ノードが
// 同時にserial_tx_<ID>へpublishすると互いの担当外スロットを0で上書きして
// しまうため)。複数モータを同時に使う場合はこのノードを拡張すること。

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

#include "damiao_ctrl/slot_layout.hpp"

namespace damiao_ctrl
{

class DamiaoDriverNode : public rclcpp::Node
{
public:
  DamiaoDriverNode()
  : Node("damiao_driver_node")
  {
    device_id_ = declare_parameter<int>("device_id", 1);
    motor_index_ = declare_parameter<int>("motor_index", 0);  // 0-origin, 0=モータ1
    const int motor_number = motor_index_ + 1;

    if (motor_index_ < 0 || static_cast<size_t>(motor_index_) >= kMaxMotors) {
      RCLCPP_FATAL(get_logger(), "motor_indexは0〜%zuの範囲で指定してください", kMaxMotors - 1);
      throw std::runtime_error("invalid motor_index");
    }

    tx_data_.fill(0);  // 全ゼロ=VELモード・target=0(安全な既定値)

    tx_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>(
      "serial_tx_" + std::to_string(device_id_), 10);
    rx_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      "serial_rx_" + std::to_string(device_id_), 10,
      std::bind(&DamiaoDriverNode::on_rx, this, std::placeholders::_1));

    state_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "damiao/motor" + std::to_string(motor_number) + "/state", 10);
    cmd_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "damiao/motor" + std::to_string(motor_number) + "/cmd", 10,
      std::bind(&DamiaoDriverNode::on_cmd, this, std::placeholders::_1));

    rclcpp::on_shutdown(std::bind(&DamiaoDriverNode::send_zero, this));

    RCLCPP_INFO(
      get_logger(), "damiao_driver_node: device_id=%d motor_index=%d (topics: damiao/motor%d/cmd, damiao/motor%d/state)",
      device_id_, motor_index_, motor_number, motor_number);
  }

private:
  void on_cmd(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) {
      RCLCPP_WARN(get_logger(), "cmdは[control_mode, target, vel_limit]の3要素が必要です");
      return;
    }
    DamiaoControlMode mode;
    if (msg->data[0] <= -0.5f) {
      mode = DamiaoControlMode::kDisabled;
    } else if (msg->data[0] >= 0.5f) {
      mode = DamiaoControlMode::kPositionVelocity;
    } else {
      mode = DamiaoControlMode::kVelocity;
    }
    set_command(tx_data_, static_cast<size_t>(motor_index_), mode, msg->data[1], msg->data[2]);
    publish_tx();
  }

  void on_rx(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != kSlotCount) {
      return;
    }
    SlotArray rx{};
    std::copy(msg->data.begin(), msg->data.end(), rx.begin());
    const auto fb = get_feedback(rx, static_cast<size_t>(motor_index_));

    std_msgs::msg::Float32MultiArray state_msg;
    state_msg.data = {
      static_cast<float>(fb.position), static_cast<float>(fb.velocity), static_cast<float>(fb.torque)};
    state_pub_->publish(state_msg);
  }

  void publish_tx()
  {
    std_msgs::msg::Int16MultiArray msg;
    msg.data.assign(tx_data_.begin(), tx_data_.end());
    tx_pub_->publish(msg);
  }

  void send_zero()
  {
    set_command(tx_data_, static_cast<size_t>(motor_index_), DamiaoControlMode::kDisabled, 0.0, 0.0);
    publish_tx();
  }

  int device_id_{1};
  int motor_index_{0};
  SlotArray tx_data_{};

  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr tx_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr rx_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub_;
};

}  // namespace damiao_ctrl

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<damiao_ctrl::DamiaoDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
