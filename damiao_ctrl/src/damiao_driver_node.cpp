// =====================================================================
// damiao_driver_node : DamiaoモータとPCの間をつなぐ「駆動ノード」
// =====================================================================
// ■ やっていること
//   自分のプログラム(PS4ノードなど)        このノード                 ros2can → ESP32 → モータ
//   damiao/motorN/cmd   を送る   ─────→  指令を24個の数値の枠に詰める ─→ serial_tx_<device_id>
//   damiao/motorN/state を受け取る ←───  返事の枠から取り出す        ←─ serial_rx_<device_id>
//   1つのノードで1枚のESP32(=1つのdevice_id、最大4台)を担当する。
//
// ■ cmd(自分のプログラムから送る。std_msgs/Float32MultiArray、data=[モード, 目標, 速さ上限])
//   [-1, 0, 0]          脱力(止める)
//   [ 0, 速度, 0]       速度制御。速度[rad/s](出力軸)
//   [ 1, 角度, 速さ]    角度制御。角度[rad]へ、速さ[rad/s]以下で回る
//   ・動かし続けるには、50回/秒くらいで送り続けること。cmd_timeout_sec(0.5秒)届かないと
//     そのモータは自動で脱力する(自分のプログラムが落ちても止まるように)。
//   ・速度と速さは ±max_vel_radps(45.6rad/s)に制限される。
//
// ■ state(このノードが送る。data=[位置, 速度, トルク, 状態, 帰還OK, 内部の位置])
//   位置[rad]: ±12.5radで一周する  速度[rad/s]  トルク[N・m]
//   状態: 0=脱力 1=有効 15=ESP32が止めている(下の「止まった後の復帰」) それ以外=故障(13=通信喪失)
//   帰還OK: 1=モータから返事が届いている 0=途絶(電源OFF・CAN線抜けなど)
//   内部の位置[rad]: モータが内部で数えている位置(不明ならNaN)。±12radを超えていると
//                    角度制御は使えない(モータの電源を入れ直すと0に戻る)
//
// ■ 止まった後の復帰(故障・状態15)
//   故障するとモータは自分で止まり、指令を出し続けても再始動しない。
//   ESP32も、PCとの通信が途絶えた・モータの返事が途絶えた・故障した時に止めて、状態を15にする
//   (通信が戻っただけでは動かない。2026-11-06 ファーム変更)。
//   どちらも、一度 [-1,0,0](脱力)を送ってから、改めて指令すると復帰する。
//   PS4テレオペならR1を一度離せばよい。
//
// ■ 【よく変える場所】config/damiao_ctrl.yaml の damiao_driver_node:
//   device_id(ESP32のDEVICE_ID)、motor_count(台数)、first_motor_number(2枚目以降のESP32用)
//
// ■ 注意: 同じdevice_idに駆動ノードを2つ起動しないこと(お互いの指令を0で上書きする)。
// =====================================================================

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

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
    const int motor_count = declare_parameter<int>("motor_count", 1);  // このESP32につないだ台数
    // このESP32の1台目のモータ番号。2枚目以降のESP32で、トピック名を続き番号にするために使う
    // (例: 2枚目に4台なら 5 → damiao/motor5〜motor8)。
    const int first_motor_number = declare_parameter<int>("first_motor_number", 1);
    cmd_timeout_sec_ = declare_parameter<double>("cmd_timeout_sec", 0.5);
    feedback_timeout_sec_ = declare_parameter<double>("feedback_timeout_sec", 0.5);
    // 速度の上限[rad/s]。説明書の無負荷最高速度は560rpm(58.6rad/s)だが、実機は
    // 48rad/s(458rpm)で頭打ちになる(2026-10-15実測。モータの最高速度レジスタRID6が
    // 1200(モータ軸rad/s)÷減速比25=48rad/sのため。2台目のESCも同じ値を確認)。
    // その95%を上限にする。
    max_vel_radps_ = declare_parameter<double>("max_vel_radps", 45.6);
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 50.0);

    if (motor_count < 1 || static_cast<size_t>(motor_count) > kMaxMotors) {
      RCLCPP_FATAL(get_logger(), "motor_countは1〜%zuの範囲で指定してください", kMaxMotors);
      throw std::runtime_error("invalid motor_count");
    }
    if (first_motor_number < 1) {
      RCLCPP_FATAL(get_logger(), "first_motor_numberは1以上にしてください");
      throw std::runtime_error("invalid first_motor_number");
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

    for (int i = 0; i < motor_count; ++i) {
      auto motor = std::make_unique<Motor>();
      motor->index = static_cast<size_t>(i);  // ESP32の中での順番(データ枠の位置)
      const int number = first_motor_number + i;  // トピック名の番号
      motor->tag = "モータ" + std::to_string(number);
      const std::string prefix = "damiao/motor" + std::to_string(number);
      motor->state_pub = create_publisher<std_msgs::msg::Float32MultiArray>(prefix + "/state", 10);
      Motor * raw = motor.get();
      motor->cmd_sub = create_subscription<std_msgs::msg::Float32MultiArray>(
        prefix + "/cmd", 10,
        [this, raw](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {on_cmd(*raw, msg);});
      motors_.push_back(std::move(motor));
    }

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
      "damiao_driver_node: device_id=%d motor_count=%d cmd_timeout=%.2fs max_vel=%.2frad/s "
      "(topics: damiao/motor%d〜%d/cmd, /state)",
      device_id_, motor_count, cmd_timeout_sec_, max_vel_radps_, first_motor_number,
      first_motor_number + motor_count - 1);
  }

  ~DamiaoDriverNode() override
  {
    // ノード破棄後にコンテキストの終了処理から呼ばれないよう登録を外す
    get_node_base_interface()->get_context()->remove_pre_shutdown_callback(shutdown_cb_handle_);
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  // モータ1台分の状態
  struct Motor
  {
    size_t index{0};  // 0-origin(スロットの位置)
    std::string tag;  // ログ用("モータ1"等)
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr state_pub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub;

    SteadyClock::time_point last_cmd_time{};
    bool cmd_timed_out{false};

    bool have_feedback{false};
    int16_t last_fb_counter{0};
    SteadyClock::time_point last_fb_change_time{};
    int16_t last_state{kMotorStateDisabled};
    bool internal_pos_large{false};
  };

  void on_cmd(Motor & motor, const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) {
      RCLCPP_WARN(
        get_logger(), "[%s] cmdは[control_mode, target, vel_limit]の3要素が必要です",
        motor.tag.c_str());
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
    double target = msg->data[1];
    double vel_limit = msg->data[2];
    // cmdはfloatなので、上限ちょうど(例: 45.6)でも誤差で僅かに超えることがある。1e-3は許容する。
    if (mode == DamiaoControlMode::kVelocity && std::abs(target) > max_vel_radps_ + 1e-3) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "[%s] 速度指令%.2frad/sを上限%.2frad/sに制限しました",
        motor.tag.c_str(), target, max_vel_radps_);
      target = std::copysign(max_vel_radps_, target);
    }
    if (mode == DamiaoControlMode::kPositionVelocity && vel_limit > max_vel_radps_ + 1e-3) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "[%s] 速度上限%.2frad/sを%.2frad/sに制限しました",
        motor.tag.c_str(), vel_limit, max_vel_radps_);
      vel_limit = max_vel_radps_;
    }
    set_command(tx_data_, motor.index, mode, target, vel_limit);
    motor.last_cmd_time = SteadyClock::now();
    if (motor.cmd_timed_out) {
      RCLCPP_INFO(get_logger(), "[%s] cmdを再び受信しました", motor.tag.c_str());
      motor.cmd_timed_out = false;
    }
    publish_tx();
  }

  void on_timer()
  {
    const auto now = SteadyClock::now();

    // 帰還途絶の警告(ros2canは届いているが、モータから返事が無い)
    for (const auto & motor : motors_) {
      if (motor->have_feedback &&
        seconds_since(motor->last_fb_change_time, now) > feedback_timeout_sec_)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[%s] モータからの帰還がありません(モータの電源・CAN線・ESC_IDを確認してください)",
          motor->tag.c_str());
      }
    }

    std::lock_guard<std::mutex> lock(tx_mutex_);
    if (shutting_down_) {
      return;
    }
    for (auto & motor : motors_) {
      const size_t off = motor_slot_offset(motor->index);
      const bool commanding =
        tx_data_[off + 0] != static_cast<int16_t>(DamiaoControlMode::kDisabled);
      // cmd途絶 -> そのモータをDISABLE
      if (commanding && seconds_since(motor->last_cmd_time, now) > cmd_timeout_sec_) {
        set_command(tx_data_, motor->index, DamiaoControlMode::kDisabled, 0.0, 0.0);
        motor->cmd_timed_out = true;
        RCLCPP_WARN(
          get_logger(), "[%s] cmdが%.2f秒以上届かないためDISABLEにしました",
          motor->tag.c_str(), cmd_timeout_sec_);
      }
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
    const auto now = SteadyClock::now();

    for (auto & motor_ptr : motors_) {
      Motor & motor = *motor_ptr;
      const auto fb = get_feedback(rx, motor.index);

      // 受信回数が進んでいれば、モータから帰還が届いている
      if (!motor.have_feedback || fb.counter != motor.last_fb_counter) {
        motor.last_fb_counter = fb.counter;
        motor.last_fb_change_time = now;
        motor.have_feedback = true;
      }
      const bool feedback_ok =
        seconds_since(motor.last_fb_change_time, now) <= feedback_timeout_sec_;

      if (feedback_ok && fb.state != motor.last_state) {
        if (is_esp32_latched(fb.state)) {
          RCLCPP_WARN(
            get_logger(),
            "[%s] ESP32がモータを止めています(状態15。起動直後・PCとの通信の途絶・モータの返事の途絶・故障の後)。"
            "一度DISABLE(-1)を送る(PS4ならR1を離す)と解除されます", motor.tag.c_str());
        } else if (is_motor_fault(fb.state)) {
          RCLCPP_ERROR(
            get_logger(),
            "[%s] モータ故障: 状態%d(%s)。モータは停止しています。復帰するには一度DISABLE(-1)を"
            "送ってから、改めて指令してください", motor.tag.c_str(), fb.state,
            motor_state_name(fb.state));
        } else {
          RCLCPP_INFO(
            get_logger(), "[%s] モータ状態: %s", motor.tag.c_str(), motor_state_name(fb.state));
        }
        motor.last_state = fb.state;
      }

      // 内部の位置が大きくなった/戻った時に知らせる
      const bool large = !std::isnan(fb.internal_pos) && std::abs(fb.internal_pos) > kPosVelMaxInternalRad;
      if (large != motor.internal_pos_large) {
        if (large) {
          RCLCPP_WARN(
            get_logger(), "[%s] モータ内部の位置が%+.1fradです(±%.0fradを超えた)。ゆっくり回した時に"
            "位置の読みが正しく変わらなくなり、角度制御は使えません。モータの電源を入れ直すと0に戻ります",
            motor.tag.c_str(), fb.internal_pos, kPosVelMaxInternalRad);
        } else {
          RCLCPP_INFO(get_logger(), "[%s] モータ内部の位置が%+.1fradに戻りました", motor.tag.c_str(),
            fb.internal_pos);
        }
        motor.internal_pos_large = large;
      }

      std_msgs::msg::Float32MultiArray state_msg;
      state_msg.data = {
        static_cast<float>(fb.position), static_cast<float>(fb.velocity),
        static_cast<float>(fb.torque), static_cast<float>(fb.state), feedback_ok ? 1.0f : 0.0f,
        static_cast<float>(fb.internal_pos)};
      motor.state_pub->publish(state_msg);
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
  // フラグを立ててそれ以降の指令を無視させる(2026-10-14実機で確認: フラグ無しだと、
  // 送り続けられていたcmdがDISABLEを上書きし、モータが回り続けた)。
  void send_zero()
  {
    {
      std::lock_guard<std::mutex> lock(tx_mutex_);
      shutting_down_ = true;
      for (const auto & motor : motors_) {
        set_command(tx_data_, motor->index, DamiaoControlMode::kDisabled, 0.0, 0.0);
      }
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
  double cmd_timeout_sec_{0.5};
  double feedback_timeout_sec_{0.5};
  double max_vel_radps_{45.6};
  SlotArray tx_data_{};
  std::mutex tx_mutex_;        // tx_data_とshutting_down_を守る
  bool shutting_down_{false};  // trueになったら以後DISABLE以外を送らない

  // unique_ptrで持つ(cmdのコールバックがMotorのアドレスを保持するため)
  std::vector<std::unique_ptr<Motor>> motors_;

  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr tx_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr rx_sub_;
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
