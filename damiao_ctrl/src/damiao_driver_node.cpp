// 1台のDamiaoモータ(DM2325+DM3520)向け高レベル駆動ノード。
// damiao/motor<N>/cmd を購読してVEL/POS_VELの指令をserial_tx_<device_id>の
// 該当6スロットへ書き込み、serial_rx_<device_id>から自分のモータ分の
// フィードバックを damiao/motor<N>/state として配信する。
// PC<->ESP32間はros2can(--nogui)を使う想定(device_idはESP32のDEVICE_ID)。
//
// cmd (std_msgs/Float32MultiArray, data=[control_mode, target, vel_limit]):
//   control_mode: -1.0=DISABLE(E-STOP), 0.0=VELモード(targetはrad/s),
//                 1.0=POS_VELモード(targetはrad, vel_limitで移動速度上限をrad/sで指定)
//   cmd_timeout_sec以上cmdが届かないと自動でDISABLEにする。ros2canの--noguiは
//   最後に受け取った指令を送り続けるため、上位ノードが落ちても止まるように
//   ここで見張る。動かし続けるには上位ノードがcmdを周期的(例: 50Hz)に送ること。
// state (std_msgs/Float32MultiArray,
//        data=[position_rad, velocity_rad_s, torque_Nm, motor_state, feedback_ok]):
//   motor_state: 0=Disable 1=Enable それ以外=故障コード(13=通信喪失など)
//   feedback_ok: 1=モータから帰還が届いている 0=途絶(電源OFF・CAN線抜け等)
//
// 故障からの復帰: モータは故障すると自分で止まり、PCが指令を出し続けても
// 再始動しない(ESP32ファームウェアとモータ側でラッチされる)。復帰させるには
// 一度DISABLE(-1)を送ってから、改めてVEL/POS_VELを送る。
//
// 同一device_idに対して駆動ノードは1つだけを想定している(複数ノードが
// 同時にserial_tx_<ID>へpublishすると互いの担当外スロットを0で上書きして
// しまうため)。複数モータを同時に使う場合はこのノードを拡張すること。

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

#include "damiao_ctrl/slot_layout.hpp"

using namespace std::chrono_literals;

namespace damiao_ctrl
{

class DamiaoDriverNode : public rclcpp::Node
{
public:
  DamiaoDriverNode()
  : Node("damiao_driver_node")
  {
    device_id_ = declare_parameter<int>("device_id", 201);
    motor_index_ = declare_parameter<int>("motor_index", 0);  // 0-origin, 0=モータ1
    cmd_timeout_sec_ = declare_parameter<double>("cmd_timeout_sec", 0.5);
    feedback_timeout_sec_ = declare_parameter<double>("feedback_timeout_sec", 0.5);
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 50.0);
    const int motor_number = motor_index_ + 1;

    if (motor_index_ < 0 || static_cast<size_t>(motor_index_) >= kMaxMotors) {
      RCLCPP_FATAL(get_logger(), "motor_indexは0〜%zuの範囲で指定してください", kMaxMotors - 1);
      throw std::runtime_error("invalid motor_index");
    }
    if (publish_rate_hz <= 0.0) {
      RCLCPP_FATAL(get_logger(), "publish_rate_hzは正の値を指定してください");
      throw std::runtime_error("invalid publish_rate_hz");
    }

    // 全ゼロ=DISABLE。起動直後からこれを周期送信するので、ros2canに前回の
    // 指令が残っていても、このノードを起動した時点でモータは止まる。
    tx_data_.fill(0);

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

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz),
      std::bind(&DamiaoDriverNode::on_timer, this));

    // 終了時(Ctrl+C)のDISABLE送信。rclcpp::on_shutdownはROSの通信を切った後に
    // 呼ばれるためpublishが届かない(2026-10-14実機で確認: 回転し続けた)。
    // 通信を切る前に呼ばれるpre-shutdownコールバックで送る。
    shutdown_cb_handle_ = get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      std::bind(&DamiaoDriverNode::send_zero, this));

    RCLCPP_INFO(
      get_logger(),
      "damiao_driver_node: device_id=%d motor_index=%d cmd_timeout=%.2fs "
      "(topics: damiao/motor%d/cmd, damiao/motor%d/state)",
      device_id_, motor_index_, cmd_timeout_sec_, motor_number, motor_number);
  }

  ~DamiaoDriverNode() override
  {
    // ノード破棄後にコンテキストの終了処理から呼ばれないよう登録を外す
    get_node_base_interface()->get_context()->remove_pre_shutdown_callback(shutdown_cb_handle_);
  }

private:
  using SteadyClock = std::chrono::steady_clock;

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
    std::lock_guard<std::mutex> lock(tx_mutex_);
    if (shutting_down_) {
      return;  // 終了処理中は上位からの指令でDISABLEを上書きさせない
    }
    set_command(tx_data_, static_cast<size_t>(motor_index_), mode, msg->data[1], msg->data[2]);
    last_cmd_time_ = SteadyClock::now();
    if (cmd_timed_out_) {
      RCLCPP_INFO(get_logger(), "cmdを再び受信しました");
      cmd_timed_out_ = false;
    }
    publish_tx();
  }

  void on_timer()
  {
    const auto now = SteadyClock::now();

    // 帰還途絶の警告(ros2canは届いているが、モータから返事が無い)
    if (have_feedback_ && seconds_since(last_fb_change_time_, now) > feedback_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "モータからの帰還がありません(モータの電源・CAN線を確認してください)");
    }

    std::lock_guard<std::mutex> lock(tx_mutex_);
    if (shutting_down_) {
      return;
    }
    const size_t off = motor_slot_offset(static_cast<size_t>(motor_index_));
    const bool commanding = tx_data_[off + 0] != static_cast<int16_t>(DamiaoControlMode::kDisabled);

    // cmd途絶 -> DISABLE
    if (commanding && seconds_since(last_cmd_time_, now) > cmd_timeout_sec_) {
      set_command(
        tx_data_, static_cast<size_t>(motor_index_), DamiaoControlMode::kDisabled, 0.0, 0.0);
      cmd_timed_out_ = true;
      RCLCPP_WARN(
        get_logger(), "cmdが%.2f秒以上届かないためDISABLEにしました", cmd_timeout_sec_);
    }

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
    const auto now = SteadyClock::now();

    // 受信回数が進んでいれば、モータから帰還が届いている
    if (!have_feedback_ || fb.counter != last_fb_counter_) {
      last_fb_counter_ = fb.counter;
      last_fb_change_time_ = now;
      have_feedback_ = true;
    }
    const bool feedback_ok = seconds_since(last_fb_change_time_, now) <= feedback_timeout_sec_;

    if (feedback_ok && fb.state != last_state_) {
      if (is_motor_fault(fb.state)) {
        RCLCPP_ERROR(
          get_logger(),
          "モータ故障: 状態%d(%s)。モータは停止しています。復帰するには一度DISABLE(-1)を"
          "送ってから、改めて指令してください", fb.state, motor_state_name(fb.state));
      } else {
        RCLCPP_INFO(get_logger(), "モータ状態: %s", motor_state_name(fb.state));
      }
      last_state_ = fb.state;
    }

    std_msgs::msg::Float32MultiArray state_msg;
    state_msg.data = {
      static_cast<float>(fb.position), static_cast<float>(fb.velocity),
      static_cast<float>(fb.torque), static_cast<float>(fb.state), feedback_ok ? 1.0f : 0.0f};
    state_pub_->publish(state_msg);
  }

  void publish_tx()
  {
    std_msgs::msg::Int16MultiArray msg;
    msg.data.assign(tx_data_.begin(), tx_data_.end());
    tx_pub_->publish(msg);
  }

  // pre-shutdownコールバック(シグナル処理のスレッドから呼ばれる)。
  // この時点ではまだエグゼキュータがon_cmd/on_timerを実行し得るため、
  // フラグを立ててそれ以降の指令を無視させる(2026-10-14実機で確認: フラグ無しだと、
  // 送り続けられていたcmdがDISABLEを上書きし、モータが回り続けた)。
  void send_zero()
  {
    {
      std::lock_guard<std::mutex> lock(tx_mutex_);
      shutting_down_ = true;
      set_command(
        tx_data_, static_cast<size_t>(motor_index_), DamiaoControlMode::kDisabled, 0.0, 0.0);
    }
    // 直後にプロセスが終了するため、取りこぼし対策として数回送り、
    // 送信が完了するまで少し待つ。
    for (int i = 0; i < 3; ++i) {
      publish_tx();
      std::this_thread::sleep_for(20ms);
    }
  }

  static double seconds_since(SteadyClock::time_point t, SteadyClock::time_point now)
  {
    return std::chrono::duration<double>(now - t).count();
  }

  int device_id_{201};
  int motor_index_{0};
  double cmd_timeout_sec_{0.5};
  double feedback_timeout_sec_{0.5};
  SlotArray tx_data_{};
  std::mutex tx_mutex_;        // tx_data_とshutting_down_を守る
  bool shutting_down_{false};  // trueになったら以後DISABLE以外を送らない

  SteadyClock::time_point last_cmd_time_{};
  bool cmd_timed_out_{false};

  bool have_feedback_{false};
  int16_t last_fb_counter_{0};
  SteadyClock::time_point last_fb_change_time_{};
  int16_t last_state_{kMotorStateDisabled};

  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr tx_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr rx_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::PreShutdownCallbackHandle shutdown_cb_handle_;
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
