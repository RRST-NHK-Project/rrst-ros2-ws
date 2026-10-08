// CubeMars AK40-10 を「出力軸rpm」「出力軸deg」で指令するためのドライバノード。
// 人やteleopノードが扱いやすい単位の指令を、ESP32(MODE_CUBEMARS)の24スロット形式へ
// 変換して serial_tx_<ID> へ50Hzでpublishし、serial_rx_<ID> の帰還を人が読める形に直す。
//
// 入力:
//   cubemars/cmd_velocity (std_msgs/Float64) 出力軸rpm。速度モードになる。
//       速度指令は velocity_cmd_timeout_sec 以内に送り続ける必要がある(途絶えたら停止)。
//   cubemars/cmd_angle    (std_msgs/Float64) 出力軸deg。角度モード(MIT)になる。
//       目標角度は保持される(送り続けなくてよい)。
//   cubemars/stop         (std_msgs/Empty)   停止(ゼロ速度指令)。
// 出力:
//   serial_tx_<ID>        (std_msgs/Int16MultiArray) ブリッジ(またはros2canのGUI)へ
//   cubemars/joint_state  (sensor_msgs/JointState) position[rad], velocity[rad/s],
//                         effort には相電流[A]を入れている(トルクではない点に注意)
//
// 設計方針:
//   - 速度制御はServoモードの速度ループ(アクチュエータ内蔵PI)をそのまま使う。
//   - 角度制御はMITモードを使う。チーム(soki)で実機動作実績があり、Kp/Kdで
//     「柔らかさ」を決められるため初回の動作確認で暴れにくい。
//   - 角度モードに入った瞬間の目標は「現在角度」にし、そこから max_angle_speed_dps で
//     目標をなだらかに動かす(位置ジャンプ防止)。目標を動かす速度をMITの目標速度にも
//     渡すことで、Kd(ダンパ)が動きを妨げて遅れるのを防ぐ。
//   - モータからの帰還が途絶えたら(受信カウンタが止まったら)停止する。帰還が無い
//     状態では角度制御の初期値も決められず、配線やIDの異常を見逃すことになるため。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

#include "cubemars_ctrl/cubemars_slots.hpp"

namespace cubemars_ctrl
{

enum class Mode { kStop, kVelocity, kAngle };

inline const char * mode_name(Mode m)
{
  switch (m) {
    case Mode::kStop: return "停止";
    case Mode::kVelocity: return "速度";
    case Mode::kAngle: return "角度";
  }
  return "?";
}

// currentをtargetへ最大stepだけ近づける。
inline double slew(double current, double target, double step)
{
  if (target > current + step) {return current + step;}
  if (target < current - step) {return current - step;}
  return target;
}

class CubemarsDriverNode : public rclcpp::Node
{
public:
  CubemarsDriverNode()
  : Node("cubemars_driver_node")
  {
    device_id_ = declare_parameter<int>("device_id", 11);
    motor_index_ = declare_parameter<int>("motor_index", 0);
    joint_name_ = declare_parameter<std::string>("joint_name", "cubemars_joint");
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 50.0);
    pole_pairs_ = declare_parameter<double>("pole_pairs", 14.0);
    gear_ratio_ = declare_parameter<double>("gear_ratio", 10.0);
    max_rpm_ = declare_parameter<double>("max_rpm", 60.0);
    rpm_slew_per_sec_ = declare_parameter<double>("rpm_slew_per_sec", 120.0);
    velocity_cmd_timeout_sec_ = declare_parameter<double>("velocity_cmd_timeout_sec", 0.5);
    angle_min_deg_ = declare_parameter<double>("angle_min_deg", -360.0);
    angle_max_deg_ = declare_parameter<double>("angle_max_deg", 360.0);
    max_angle_speed_dps_ = declare_parameter<double>("max_angle_speed_dps", 90.0);
    mit_kp_ = declare_parameter<double>("mit_kp", 3.6);
    mit_kd_ = declare_parameter<double>("mit_kd", 1.08);
    angle_entry_max_rpm_ = declare_parameter<double>("angle_entry_max_rpm", 10.0);
    feedback_timeout_sec_ = declare_parameter<double>("feedback_timeout_sec", 0.5);
    log_period_sec_ = declare_parameter<double>("log_period_sec", 0.5);

    validate_parameters();

    tx_.fill(0);

    const std::string tx_topic = "serial_tx_" + std::to_string(device_id_);
    const std::string rx_topic = "serial_rx_" + std::to_string(device_id_);
    tx_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>(tx_topic, 10);
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("cubemars/joint_state", 10);

    rx_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      rx_topic, rclcpp::SensorDataQoS(),
      std::bind(&CubemarsDriverNode::on_feedback, this, std::placeholders::_1));
    vel_sub_ = create_subscription<std_msgs::msg::Float64>(
      "cubemars/cmd_velocity", 10,
      std::bind(&CubemarsDriverNode::on_cmd_velocity, this, std::placeholders::_1));
    angle_sub_ = create_subscription<std_msgs::msg::Float64>(
      "cubemars/cmd_angle", 10,
      std::bind(&CubemarsDriverNode::on_cmd_angle, this, std::placeholders::_1));
    stop_sub_ = create_subscription<std_msgs::msg::Empty>(
      "cubemars/stop", 10,
      [this](std_msgs::msg::Empty::SharedPtr) {
        if (mode_ != Mode::kStop) {
          RCLCPP_INFO(get_logger(), "停止指令を受信しました。");
        }
        enter_stop();
      });

    dt_ = 1.0 / publish_rate_hz;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&CubemarsDriverNode::on_timer, this));

    // ブリッジ側・ESP32側にもフェイルセーフはあるが、正常終了時はここで明示的に止める。
    // (pre-shutdownで呼ばないとコンテキストが無効になった後でpublishできない)
    get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      [this]() {publish_stop_frame();});

    RCLCPP_INFO(
      get_logger(),
      "cubemars_driver_node 起動: device_id=%d motor_index=%d (M%d) "
      "極対数=%.1f 減速比=%.1f (1rpm=%.0fERPM) 最大%.1frpm 角度範囲[%.1f, %.1f]deg "
      "MIT Kp=%.2f Kd=%.2f",
      device_id_, motor_index_, motor_index_ + 1, pole_pairs_, gear_ratio_,
      output_rpm_to_erpm(1.0, pole_pairs_, gear_ratio_), max_rpm_,
      angle_min_deg_, angle_max_deg_, mit_kp_, mit_kd_);
  }

private:
  void validate_parameters()
  {
    if (motor_index_ < 0 || motor_index_ >= static_cast<int>(kMaxMotors)) {
      throw std::invalid_argument("motor_index は 0〜3 で指定してください");
    }
    if (pole_pairs_ <= 0.0 || gear_ratio_ <= 0.0) {
      throw std::invalid_argument("pole_pairs / gear_ratio は正の値にしてください");
    }
    if (angle_min_deg_ >= angle_max_deg_) {
      throw std::invalid_argument("angle_min_deg < angle_max_deg にしてください");
    }
    max_rpm_ = std::max(0.0, max_rpm_);
    // MITの位置はプロトコル上±716.2deg(±12.5rad)が限界。範囲外は無警告で
    // クランプされ「そこで止まる」ので、少し内側までに制限する。
    const double mit_limit = kMitPositionLimitDeg - kMitMarginDeg;
    if (angle_min_deg_ < -mit_limit || angle_max_deg_ > mit_limit) {
      RCLCPP_WARN(
        get_logger(),
        "角度範囲[%.1f, %.1f]degはMITモードの限界(±%.1fdeg)を超えるため切り詰めます。",
        angle_min_deg_, angle_max_deg_, mit_limit);
      angle_min_deg_ = std::max(angle_min_deg_, -mit_limit);
      angle_max_deg_ = std::min(angle_max_deg_, mit_limit);
    }
    // MITの目標速度もプロトコル上限(45.5rad/s ≈ 2607deg/s)を超えないようにする。
    max_angle_speed_dps_ = std::clamp(
      max_angle_speed_dps_, 1.0, rad_to_deg(kMitVelocityLimitRadps));
    mit_kp_ = std::clamp(mit_kp_, 0.0, kMitKpMax);
    mit_kd_ = std::clamp(mit_kd_, 0.0, kMitKdMax);
  }

  // ---------------- 入力 ----------------

  void on_cmd_velocity(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cmd_velocityがNaN/infです。無視します。");
      return;
    }
    if (!feedback_alive()) {
      warn_no_feedback("速度指令");
      return;
    }
    cmd_rpm_ = msg->data;
    last_vel_cmd_ = now();
    if (mode_ != Mode::kVelocity) {
      mode_ = Mode::kVelocity;
      current_rpm_ = 0.0;  // 0から加速させる(急発進防止)
      RCLCPP_INFO(get_logger(), "速度モードに入りました。");
    }
  }

  void on_cmd_angle(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cmd_angleがNaN/infです。無視します。");
      return;
    }
    if (mode_ != Mode::kAngle && !try_enter_angle_mode()) {
      return;
    }
    const double clamped = std::clamp(msg->data, angle_min_deg_, angle_max_deg_);
    if (clamped != msg->data) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "目標角度 %.1fdeg は範囲[%.1f, %.1f]外のため %.1fdeg にします。",
        msg->data, angle_min_deg_, angle_max_deg_, clamped);
    }
    cmd_angle_deg_ = clamped;
  }

  bool try_enter_angle_mode()
  {
    if (!feedback_alive() || !fb_) {
      warn_no_feedback("角度指令");
      return false;
    }
    const double pos = fb_->position_deg;
    const double mit_limit = kMitPositionLimitDeg - kMitMarginDeg;
    if (std::abs(pos) > mit_limit) {
      // 現在角度がMITで表現できない位置にあると、目標がクランプされた位置へ
      // 向かって勝手に動き出す。安全側で拒否する。
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "現在角度 %.1fdeg がMITモードの範囲(±%.1fdeg)外のため角度モードに入れません。"
        "速度モードで0deg付近まで戻してください。", pos, mit_limit);
      return false;
    }
    const double rpm = measured_rpm();
    if (std::abs(rpm) > angle_entry_max_rpm_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "回転中(%.1frpm)は角度モードに入れません。先に停止してください。", rpm);
      return false;
    }
    mode_ = Mode::kAngle;
    setpoint_deg_ = pos;  // 現在角度から開始(位置ジャンプ防止)
    cmd_angle_deg_ = pos;
    RCLCPP_INFO(get_logger(), "角度モードに入りました(開始角度 %.1fdeg)。", pos);
    return true;
  }

  void on_feedback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < kSlotCount) {
      return;
    }
    std::copy(msg->data.begin(), msg->data.begin() + kSlotCount, rx_.begin());
    const MotorFeedback fb = decode_feedback(rx_, static_cast<size_t>(motor_index_));
    const rclcpp::Time t = now();

    // 受信カウンタが変化した = ESP32がモータの帰還フレーム(0x29)を新たに受け取った。
    if (!last_counter_ || fb.feedback_counter != *last_counter_) {
      if (last_counter_) {
        counter_increments_ += static_cast<uint16_t>(fb.feedback_counter - *last_counter_);
        last_feedback_change_ = t;
        fb_seen_ = true;
      }
      last_counter_ = fb.feedback_counter;
    }

    // 位置の変化からも出力軸rpmを推定する(極対数・減速比の設定が正しいかの確認用)。
    if (fb_ && fb_seen_) {
      const double d_pos = fb.position_deg - fb_->position_deg;
      // 帰還の位置は±3200deg付近で折り返すため、大きな飛びは推定に使わない。
      if (std::abs(d_pos) < 1000.0) {
        pos_rate_accum_deg_ += d_pos;
      }
    }
    speed_erpm_sum_ += fb.speed_erpm;
    ++speed_samples_;

    if (fb.error_code != 0 && fb.error_code != last_error_code_) {
      RCLCPP_ERROR(
        get_logger(), "モータがエラーを報告しました: %d (%s)。停止します。",
        fb.error_code, error_text(fb.error_code));
      enter_stop();
    } else if (fb.error_code == 0 && last_error_code_ != 0) {
      RCLCPP_INFO(get_logger(), "モータのエラーが解除されました。");
    }
    last_error_code_ = fb.error_code;
    fb_ = fb;

    sensor_msgs::msg::JointState js;
    js.header.stamp = t;
    js.name.push_back(joint_name_);
    js.position.push_back(deg_to_rad(fb.position_deg));
    js.velocity.push_back(measured_rpm() * 2.0 * kPi / 60.0);
    js.effort.push_back(fb.current_a);
    joint_pub_->publish(js);
  }

  // ---------------- 周期処理 ----------------

  void on_timer()
  {
    const rclcpp::Time t = now();
    const size_t m = static_cast<size_t>(motor_index_);

    if (mode_ != Mode::kStop && !feedback_alive()) {
      RCLCPP_ERROR(
        get_logger(),
        "モータからの帰還が %.1f 秒途絶えたため停止します(CAN配線・モータ電源を確認)。",
        feedback_timeout_sec_);
      enter_stop();
    }

    if (mode_ == Mode::kVelocity &&
      (t - last_vel_cmd_).seconds() > velocity_cmd_timeout_sec_)
    {
      RCLCPP_INFO(get_logger(), "速度指令が %.1f 秒途絶えたため停止します。", velocity_cmd_timeout_sec_);
      enter_stop();
    }

    switch (mode_) {
      case Mode::kStop:
        set_stop(tx_, m);
        break;
      case Mode::kVelocity: {
          const double desired = std::clamp(cmd_rpm_, -max_rpm_, max_rpm_);
          current_rpm_ = slew(current_rpm_, desired, rpm_slew_per_sec_ * dt_);
          set_velocity_erpm(tx_, m, output_rpm_to_erpm(current_rpm_, pole_pairs_, gear_ratio_));
          break;
        }
      case Mode::kAngle: {
          const double prev = setpoint_deg_;
          setpoint_deg_ = slew(setpoint_deg_, cmd_angle_deg_, max_angle_speed_dps_ * dt_);
          const double vel_ff_radps = deg_to_rad((setpoint_deg_ - prev) / dt_);
          set_mit(tx_, m, setpoint_deg_, vel_ff_radps, mit_kp_, mit_kd_);
          break;
        }
    }

    std_msgs::msg::Int16MultiArray msg;
    msg.data.assign(tx_.begin(), tx_.end());
    tx_pub_->publish(msg);

    log_status(t);
  }

  void log_status(const rclcpp::Time & t)
  {
    if (!last_log_) {
      last_log_ = t;
      return;
    }
    const double elapsed = (t - *last_log_).seconds();
    if (elapsed < log_period_sec_) {
      return;
    }
    last_log_ = t;

    const double fb_rate_hz = counter_increments_ / elapsed;
    const double rpm_from_pos = pos_rate_accum_deg_ / elapsed / 6.0;  // deg/s -> rpm
    const double avg_erpm = speed_samples_ > 0 ? speed_erpm_sum_ / speed_samples_ : 0.0;
    counter_increments_ = 0;
    pos_rate_accum_deg_ = 0.0;
    speed_erpm_sum_ = 0.0;
    speed_samples_ = 0;

    if (!fb_seen_ || !fb_) {
      RCLCPP_WARN(
        get_logger(),
        "[%s] モータからの帰還がまだありません。ブリッジ/ESP32が動いているか"
        "(ros2 topic hz /serial_rx_%d)、モータの電源・CAN配線・CAN IDを確認してください。",
        mode_name(mode_), device_id_);
      return;
    }

    std::string target;
    if (mode_ == Mode::kVelocity) {
      target = "指令=" + fmt(current_rpm_, 1) + "rpm";
    } else if (mode_ == Mode::kAngle) {
      target = "目標=" + fmt(cmd_angle_deg_, 1) + "deg(現在の目標" + fmt(setpoint_deg_, 1) + "deg)";
    } else {
      target = "指令=0";
    }

    // 回転中は同じ区間の「速度帰還(ERPM)の平均」と「位置の変化から求めた出力軸rpm」の比から
    // 極対数×減速比 を逆算して表示する(設定値と大きく違えばpole_pairs/gear_ratioが誤り)。
    std::string ratio;
    if (std::abs(rpm_from_pos) > 5.0) {
      ratio = " ERPM/出力rpm比=" + fmt(avg_erpm / rpm_from_pos, 0) +
        "(設定値" + fmt(pole_pairs_ * gear_ratio_, 0) + ")";
    }

    char line[512];
    std::snprintf(
      line, sizeof(line),
      "[%s] %s | 位置=%.1fdeg 速度=%.1frpm(位置の変化から%.1frpm) 電流=%.2fA 温度=%dC "
      "エラー=%s 帰還=%.0fHz%s",
      mode_name(mode_), target.c_str(), fb_->position_deg, measured_rpm(), rpm_from_pos,
      fb_->current_a, fb_->temperature_c, error_text(fb_->error_code), fb_rate_hz, ratio.c_str());
    if (feedback_alive()) {
      RCLCPP_INFO(get_logger(), "%s", line);
    } else {
      RCLCPP_WARN(get_logger(), "%s (帰還途絶中)", line);
    }
  }

  // ---------------- ヘルパ ----------------

  bool feedback_alive() const
  {
    return fb_seen_ && (now() - last_feedback_change_).seconds() < feedback_timeout_sec_;
  }

  void warn_no_feedback(const char * what)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "モータからの帰還が無いため%sを無視します(ブリッジ・ESP32・モータ電源・CAN IDを確認)。", what);
  }

  double measured_rpm() const
  {
    return fb_ ? erpm_to_output_rpm(fb_->speed_erpm, pole_pairs_, gear_ratio_) : 0.0;
  }

  void enter_stop()
  {
    mode_ = Mode::kStop;
    current_rpm_ = 0.0;
    cmd_rpm_ = 0.0;
  }

  void publish_stop_frame()
  {
    set_stop(tx_, static_cast<size_t>(motor_index_));
    std_msgs::msg::Int16MultiArray msg;
    msg.data.assign(tx_.begin(), tx_.end());
    tx_pub_->publish(msg);
  }

  static std::string fmt(double v, int digits)
  {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.*f", digits, v);
    return buf;
  }

  static constexpr double kMitMarginDeg = 16.2;  // 716.2 -> 700deg

  // パラメータ
  int device_id_{11};
  int motor_index_{0};
  std::string joint_name_;
  double dt_{0.02};
  double pole_pairs_{14.0};
  double gear_ratio_{10.0};
  double max_rpm_{60.0};
  double rpm_slew_per_sec_{120.0};
  double velocity_cmd_timeout_sec_{0.5};
  double angle_min_deg_{-360.0};
  double angle_max_deg_{360.0};
  double max_angle_speed_dps_{90.0};
  double mit_kp_{3.6};
  double mit_kd_{1.08};
  double angle_entry_max_rpm_{10.0};
  double feedback_timeout_sec_{0.5};
  double log_period_sec_{0.5};

  // 状態
  Mode mode_{Mode::kStop};
  SlotArray tx_{};
  SlotArray rx_{};
  double cmd_rpm_{0.0};
  double current_rpm_{0.0};
  rclcpp::Time last_vel_cmd_{0, 0, RCL_ROS_TIME};
  double cmd_angle_deg_{0.0};
  double setpoint_deg_{0.0};

  std::optional<MotorFeedback> fb_;
  std::optional<int16_t> last_counter_;
  bool fb_seen_{false};
  rclcpp::Time last_feedback_change_{0, 0, RCL_ROS_TIME};
  int last_error_code_{0};
  uint32_t counter_increments_{0};
  double pos_rate_accum_deg_{0.0};
  double speed_erpm_sum_{0.0};
  uint32_t speed_samples_{0};
  std::optional<rclcpp::Time> last_log_;

  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr tx_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr rx_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cubemars_ctrl

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cubemars_ctrl::CubemarsDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
