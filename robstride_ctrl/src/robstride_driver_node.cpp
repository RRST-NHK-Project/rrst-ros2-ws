// 1台のRobStride EduLite 05 向け高レベル駆動ノード(damiao_ctrlのdamiao_driver_nodeと同じ作り)。
// robstride/motor<N>/cmd を購読して速度/位置の指令をserial_tx_<device_id>の該当8スロットへ
// 書き込み、serial_rx_<device_id>から自分のモータ分の帰還を robstride/motor<N>/state として配信する。
// PC<->ESP32間はros2can(--nogui)を使う想定(device_idはESP32のDEVICE_ID)。
//
// cmd (std_msgs/Float32MultiArray, data=[control_mode, target, vel_limit]):
//   control_mode: -1.0=DISABLE(E-STOP), 0.0=速度モード(targetはrad/s),
//                 1.0=位置モード(targetは出力軸の多回転位置rad、vel_limitで移動速度の上限rad/s)
//   速度のtargetと位置のvel_limitは±max_vel_radpsに制限する。
//   位置モードは、モータの位置を読めていない間・帰還が途絶えている間は受け付けない
//   (目標が正しい座標か確かめられないため)。
//   cmd_timeout_sec以上cmdが届かないと自動でDISABLEにする(ros2canの--noguiは
//   最後の指令を送り続けるため、上位ノードが落ちても止まるようにここで見張る)。
// state (std_msgs/Float32MultiArray,
//        data=[position_rad, velocity_rad_s, torque_Nm, motor_mode, feedback_ok,
//              fault_flags, temperature_degC]):
//   motor_mode: 0=停止 1=校正中 2=運転
//   fault_flags: 下位6bit=モータの故障、256=ESP32が止めた(ラッチ)、512=位置未取得
//
// 異常からの復帰: 運転中にモータが止まった(故障・CAN途絶・電源断)場合、ESP32が止めたまま
// ラッチし、PCが指令を出し続けても再始動しない。一度DISABLE(-1)を送ってから改めて指令する。
//
// 同一device_idに対して駆動ノードは1つだけを想定している(複数ノードが同時に
// serial_tx_<ID>へpublishすると互いの担当外スロットを0で上書きしてしまうため)。

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

#include "robstride_ctrl/slot_layout.hpp"

using namespace std::chrono_literals;

namespace robstride_ctrl
{

class RobstrideDriverNode : public rclcpp::Node
{
public:
  RobstrideDriverNode()
  : Node("robstride_driver_node")
  {
    device_id_ = declare_parameter<int>("device_id", 202);
    motor_index_ = declare_parameter<int>("motor_index", 0);  // 0-origin, 0=モータ1
    cmd_timeout_sec_ = declare_parameter<double>("cmd_timeout_sec", 0.5);
    feedback_timeout_sec_ = declare_parameter<double>("feedback_timeout_sec", 0.5);
    // 速度の上限[rad/s]。EL05の無負荷最高速度は48V時に約45rad/s(430rpm)、24V時は約23.5rad/s
    // (参考資料の実測)。まずは24Vで頭打ちになる手前の20rad/sにしておき、実測後に見直す。
    max_vel_radps_ = declare_parameter<double>("max_vel_radps", 20.0);
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
      std::bind(&RobstrideDriverNode::on_rx, this, std::placeholders::_1));

    state_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "robstride/motor" + std::to_string(motor_number) + "/state", 10);
    cmd_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "robstride/motor" + std::to_string(motor_number) + "/cmd", 10,
      std::bind(&RobstrideDriverNode::on_cmd, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz),
      std::bind(&RobstrideDriverNode::on_timer, this));

    // 終了時(Ctrl+C)のDISABLE送信。rclcpp::on_shutdownはROSの通信を切った後に
    // 呼ばれるためpublishが届かない(damiao_ctrlで2026-10-14に実機で確認)。
    // 通信を切る前に呼ばれるpre-shutdownコールバックで送る。
    shutdown_cb_handle_ = get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      std::bind(&RobstrideDriverNode::send_disable_on_shutdown, this));

    RCLCPP_INFO(
      get_logger(),
      "robstride_driver_node: device_id=%d motor_index=%d cmd_timeout=%.2fs max_vel=%.2frad/s "
      "(topics: robstride/motor%d/cmd, robstride/motor%d/state)",
      device_id_, motor_index_, cmd_timeout_sec_, max_vel_radps_, motor_number, motor_number);
  }

  ~RobstrideDriverNode() override
  {
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
    RobstrideControlMode mode;
    if (msg->data[0] <= -0.5f) {
      mode = RobstrideControlMode::kDisabled;
    } else if (msg->data[0] >= 0.5f) {
      mode = RobstrideControlMode::kPosition;
    } else {
      mode = RobstrideControlMode::kVelocity;
    }
    double target = msg->data[1];
    double vel_limit = msg->data[2];
    if (!std::isfinite(target) || !std::isfinite(vel_limit)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "cmdにNaN/無限大が含まれているため無視しました");
      return;
    }

    std::lock_guard<std::mutex> lock(tx_mutex_);
    if (shutting_down_) {
      return;  // 終了処理中は上位からの指令でDISABLEを上書きさせない
    }
    if (mode == RobstrideControlMode::kPosition && !position_ready_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "モータの位置をまだ読めていない(または帰還が途絶えている)ため、位置指令をDISABLEに"
        "置き換えました");
      mode = RobstrideControlMode::kDisabled;
    }
    // cmdはfloatなので、上限ちょうどでも誤差で僅かに超えることがある。1e-3は許容する。
    if (mode == RobstrideControlMode::kVelocity && std::abs(target) > max_vel_radps_ + 1e-3) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "速度指令%.2frad/sを上限%.2frad/sに制限しました",
        target, max_vel_radps_);
      target = std::copysign(max_vel_radps_, target);
    }
    if (mode == RobstrideControlMode::kPosition) {
      if (vel_limit <= 0.0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "位置モードはvel_limit(移動速度の上限rad/s)に正の値が必要です。DISABLEにしました");
        mode = RobstrideControlMode::kDisabled;
      } else if (vel_limit > max_vel_radps_ + 1e-3) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "速度上限%.2frad/sを%.2frad/sに制限しました",
          vel_limit, max_vel_radps_);
        vel_limit = max_vel_radps_;
      }
    }
    if (mode == RobstrideControlMode::kDisabled) {
      target = 0.0;
      vel_limit = 0.0;
    }
    set_command(tx_data_, static_cast<size_t>(motor_index_), mode, target, vel_limit);
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

    if (have_feedback_ && seconds_since(last_fb_change_time_, now) > feedback_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "モータからの帰還がありません(モータの電源・CAN線・config.hppのモータIDを確認)");
    }

    std::lock_guard<std::mutex> lock(tx_mutex_);
    if (shutting_down_) {
      return;
    }
    const size_t off = motor_slot_offset(static_cast<size_t>(motor_index_));
    const auto current_mode = static_cast<RobstrideControlMode>(tx_data_[off + 0]);
    const bool commanding = current_mode != RobstrideControlMode::kDisabled;

    // cmd途絶 -> DISABLE
    if (commanding && seconds_since(last_cmd_time_, now) > cmd_timeout_sec_) {
      set_command(
        tx_data_, static_cast<size_t>(motor_index_), RobstrideControlMode::kDisabled, 0.0, 0.0);
      cmd_timed_out_ = true;
      RCLCPP_WARN(get_logger(), "cmdが%.2f秒以上届かないためDISABLEにしました", cmd_timeout_sec_);
    }
    // 位置モード中に位置が読めなくなった -> DISABLE
    if (current_mode == RobstrideControlMode::kPosition && !position_ready_) {
      set_command(
        tx_data_, static_cast<size_t>(motor_index_), RobstrideControlMode::kDisabled, 0.0, 0.0);
      RCLCPP_WARN(get_logger(), "位置モード中に帰還が途絶えたためDISABLEにしました");
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

    if (!have_feedback_ || fb.counter != last_fb_counter_) {
      last_fb_counter_ = fb.counter;
      last_fb_change_time_ = now;
      have_feedback_ = true;
    }
    const bool feedback_ok = seconds_since(last_fb_change_time_, now) <= feedback_timeout_sec_;
    {
      std::lock_guard<std::mutex> lock(tx_mutex_);
      position_ready_ = feedback_ok && !(fb.flags & kFlagPositionUnknown);
    }

    report_changes(fb, feedback_ok);
    report_scan(rx);

    std_msgs::msg::Float32MultiArray state_msg;
    state_msg.data = {
      static_cast<float>(fb.position), static_cast<float>(fb.velocity),
      static_cast<float>(fb.torque), static_cast<float>(fb.mode), feedback_ok ? 1.0f : 0.0f,
      static_cast<float>(fb.flags), static_cast<float>(fb.temperature)};
    state_pub_->publish(state_msg);
  }

  // モードや故障が変わった時だけログに出す
  void report_changes(const MotorFeedback & fb, bool feedback_ok)
  {
    if (!feedback_ok) {
      return;
    }
    if (fb.mode != last_mode_) {
      RCLCPP_INFO(get_logger(), "モータのモード: %s", motor_mode_name(fb.mode));
      last_mode_ = fb.mode;
    }
    if (fb.flags != last_flags_) {
      const uint16_t faults = fb.flags & kFaultMask;
      if (faults) {
        std::ostringstream names;
        for (const char * n : fault_names(faults)) {
          names << n << " ";
        }
        RCLCPP_ERROR(get_logger(), "モータ故障: %s(0x%02x)", names.str().c_str(), faults);
      }
      if ((fb.flags & kFlagLatched) && !(last_flags_ & kFlagLatched)) {
        RCLCPP_ERROR(
          get_logger(),
          "運転中にモータが止まったため、ESP32が停止を保持しています。復帰するには一度"
          "DISABLE(-1)を送ってから、改めて指令してください");
      }
      if (!(fb.flags & kFlagPositionUnknown) && (last_flags_ & kFlagPositionUnknown)) {
        RCLCPP_INFO(get_logger(), "モータの位置を読めました: %.3f rad", fb.position);
      }
      last_flags_ = fb.flags;
    }
  }

  // ESP32のIDスキャン結果(診断スロット)が変わったらログに出す
  void report_scan(const SlotArray & rx)
  {
    const int16_t count = rx[kDiagSlot];
    std::ostringstream ids;
    for (size_t i = 0; i < kDiagIdSlots; ++i) {
      if (rx[kDiagSlot + 1 + i] >= 0) {
        ids << rx[kDiagSlot + 1 + i] << " ";
      }
    }
    const std::string text = std::to_string(count) + ":" + ids.str();
    if (text == last_scan_text_) {
      return;
    }
    last_scan_text_ = text;
    if (count > 0) {
      RCLCPP_INFO(
        get_logger(), "ESP32のIDスキャン: %d台応答 (CAN ID: %s)", count, ids.str().c_str());
    }
  }

  void publish_tx()
  {
    std_msgs::msg::Int16MultiArray msg;
    msg.data.assign(tx_data_.begin(), tx_data_.end());
    tx_pub_->publish(msg);
  }

  // pre-shutdownコールバック(シグナル処理のスレッドから呼ばれる)。
  // この時点ではまだエグゼキュータがon_cmd/on_timerを実行し得るため、
  // フラグを立ててそれ以降の指令を無視させる。
  void send_disable_on_shutdown()
  {
    {
      std::lock_guard<std::mutex> lock(tx_mutex_);
      shutting_down_ = true;
      set_command(
        tx_data_, static_cast<size_t>(motor_index_), RobstrideControlMode::kDisabled, 0.0, 0.0);
    }
    for (int i = 0; i < 3; ++i) {
      publish_tx();
      std::this_thread::sleep_for(20ms);
    }
  }

  static double seconds_since(SteadyClock::time_point t, SteadyClock::time_point now)
  {
    return std::chrono::duration<double>(now - t).count();
  }

  int device_id_{202};
  int motor_index_{0};
  double cmd_timeout_sec_{0.5};
  double feedback_timeout_sec_{0.5};
  double max_vel_radps_{20.0};
  SlotArray tx_data_{};
  std::mutex tx_mutex_;        // tx_data_, shutting_down_, position_ready_を守る
  bool shutting_down_{false};  // trueになったら以後DISABLE以外を送らない
  bool position_ready_{false}; // 帰還があり、位置(mechPos)を読めている

  SteadyClock::time_point last_cmd_time_{};
  bool cmd_timed_out_{false};

  bool have_feedback_{false};
  int16_t last_fb_counter_{0};
  SteadyClock::time_point last_fb_change_time_{};
  int16_t last_mode_{-1};
  uint16_t last_flags_{0};
  std::string last_scan_text_;

  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr tx_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr rx_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::PreShutdownCallbackHandle shutdown_cb_handle_;
};

}  // namespace robstride_ctrl

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robstride_ctrl::RobstrideDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
