// PS4コントローラ(USB有線, joyパッケージ経由)でRobStride EduLite 05の
// 速度制御(速度モード)をテレオペするノード。
// damiao_ctrlで2026-10-14に実機確認済みのps4_teleop_nodeと同じ安全設計・ボタン配置で、
// 送り先のトピック(robstride/motor<N>/cmd)だけが異なる。
//
// 安全設計:
//   - デッドマンスイッチ: R1(既定)を押している間だけ指令を有効化する。
//     R1を離すと即座にDISABLE(-1)を送る。
//   - 起動直後・E-STOP後・joy途絶後は、R1を一度離してから押し直すまで
//     有効化しない(R1を押したまま起動/復帰しても急に回らないように)。
//   - E-STOP: Options(既定)を押すとDISABLE。上記の通りR1の押し直しが必要。
//   - joy途絶: /joyがjoy_timeout_sec以上届かないとDISABLE(コントローラの
//     USB抜け・joy_nodeの停止時用。joy_nodeは入力に変化が無くても
//     autorepeat_rate(既定20Hz)で/joyを送り直すため、途絶=異常と判断できる)。
//   - /joyの送信元が2つ以上ある(他PCのjoy_node等が混ざっている)場合は停止し、
//     送信元が1つに戻ってもR1の押し直しが必要。
//   - このノード自体が落ちた場合はrobstride_driver_nodeのcmd途絶の見張りで止まる。
//   - モータが故障(通信喪失等)で止まった場合は、R1を離して押し直すと
//     DISABLE->再有効化の順になり復帰する。
//
// ボタン/軸の番号(2026-10-14 実機確認: DualShock 4 CUH-ZCT1をUSB接続、joy 3.3.0、
// Linuxのplaystationドライバ。軸8本・ボタン13個):
//   buttons: 0=×, 1=○, 2=△, 3=□, 4=L1, 5=R1, 6=L2, 7=R2, 8=Share, 9=Options,
//            10=PS, 11=L3, 12=R3
//   axes:    0=左スティック左右, 1=左スティック上下(上が+1.0), 2=L2(離して+1.0),
//            3=右スティック左右, 4=右スティック上下, 5=R2(離して+1.0), 6/7=十字キー
// 別のコントローラ・接続方法では番号が変わることがあるので、
// `ros2 topic echo /joy` で必ず確認すること(README参照)。

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

namespace robstride_ctrl
{

class Ps4TeleopNode : public rclcpp::Node
{
public:
  Ps4TeleopNode()
  : Node("ps4_teleop_node")
  {
    axis_index_ = declare_parameter<int>("velocity_axis_index", 1);         // 左スティック上下
    deadman_button_index_ = declare_parameter<int>("deadman_button_index", 5);   // R1
    estop_button_index_ = declare_parameter<int>("estop_button_index", 9);     // Options
    max_vel_radps_ = declare_parameter<double>("max_vel_radps", 1.0);       // 初回動作確認用に控えめな値
    deadzone_ = declare_parameter<double>("deadzone", 0.08);
    invert_axis_ = declare_parameter<bool>("invert_axis", false);           // 回転方向を逆にする
    joy_timeout_sec_ = declare_parameter<double>("joy_timeout_sec", 0.5);
    motor_number_ = declare_parameter<int>("motor_number", 1);

    cmd_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "robstride/motor" + std::to_string(motor_number_) + "/cmd", 10);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Ps4TeleopNode::on_joy, this, std::placeholders::_1));

    // 50Hzで最新の指令を送る(robstride_driver_nodeのcmd途絶の見張りより十分速い)。
    publish_timer_ = create_wall_timer(20ms, std::bind(&Ps4TeleopNode::publish_cmd, this));

    // 終了時(Ctrl+C)はROSの通信を切る前にDISABLEを送る(robstride_driver_nodeと同じ理由で
    // rclcpp::on_shutdownは使わない)。
    shutdown_cb_handle_ = get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      std::bind(&Ps4TeleopNode::send_disable_on_shutdown, this));

    RCLCPP_INFO(
      get_logger(),
      "ps4_teleop_node: axis=%d deadman_button=%d estop_button=%d max_vel=%.2frad/s "
      "joy_timeout=%.2fs (R1を押している間だけ動く。実際のボタン配置は "
      "`ros2 topic echo /joy` で要確認)",
      axis_index_, deadman_button_index_, estop_button_index_, max_vel_radps_, joy_timeout_sec_);
  }

  ~Ps4TeleopNode() override
  {
    get_node_base_interface()->get_context()->remove_pre_shutdown_callback(shutdown_cb_handle_);
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_time_ = SteadyClock::now();
    have_joy_ = true;
    if (multiple_joy_publishers_) {
      return;  // どの/joyが本物か分からないので、R1の状態も含めて一切使わない
    }

    const bool axis_ok = axis_index_ >= 0 && static_cast<size_t>(axis_index_) < msg->axes.size();
    const bool deadman_ok =
      deadman_button_index_ >= 0 && static_cast<size_t>(deadman_button_index_) < msg->buttons.size();
    const bool estop_ok =
      estop_button_index_ >= 0 && static_cast<size_t>(estop_button_index_) < msg->buttons.size();
    if (!axis_ok || !deadman_ok || !estop_ok) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "/joyの軸/ボタン数(axes=%zu, buttons=%zu)が設定した番号に足りません。"
        "config/robstride_ctrl.yamlを確認してください", msg->axes.size(), msg->buttons.size());
      set_enabled(false, "設定エラー");
      armed_ = false;
      return;
    }

    const bool estop_pressed = msg->buttons[estop_button_index_] != 0;
    const bool deadman_held = msg->buttons[deadman_button_index_] != 0;

    if (estop_pressed) {
      if (armed_ || enabled_) {
        RCLCPP_WARN(get_logger(), "E-STOP: 停止しました(R1を離して押し直すと再開)");
      }
      armed_ = false;
      set_enabled(false, "E-STOP");
      return;
    }
    if (!deadman_held) {
      if (!armed_) {
        RCLCPP_INFO(get_logger(), "準備完了: R1を押している間だけ動きます");
      }
      armed_ = true;  // R1を離した状態を確認できたので、次の押下で有効化してよい
      set_enabled(false, "R1を離した");
      return;
    }
    if (!armed_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "R1を一度離してから押し直してください");
      return;
    }

    double raw = static_cast<double>(msg->axes[axis_index_]);
    if (std::abs(raw) < deadzone_) {
      raw = 0.0;
    }
    target_vel_ = (invert_axis_ ? -raw : raw) * max_vel_radps_;
    set_enabled(true, "R1押下");
  }

  void publish_cmd()
  {
    // /joyの送信元が複数 -> 停止(2026-10-15: 同じWi-Fiの他PCのjoy_nodeが混ざり、
    // R1が押す/離すを繰り返した)
    const size_t joy_publishers = count_publishers(joy_sub_->get_topic_name());
    multiple_joy_publishers_ = joy_publishers > 1;
    if (multiple_joy_publishers_) {
      if (armed_ || enabled_) {
        RCLCPP_ERROR(
          get_logger(), "/joyの送信元が%zu個あります(他のPCのjoy_node等)。停止しました。"
          "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOSTで起動してください", joy_publishers);
      }
      armed_ = false;
      set_enabled(false, "/joyの送信元が複数");
    }

    // joy途絶 -> 停止(R1の押し直しが必要)
    if (have_joy_ &&
      std::chrono::duration<double>(SteadyClock::now() - last_joy_time_).count() > joy_timeout_sec_)
    {
      if (armed_ || enabled_) {
        RCLCPP_WARN(
          get_logger(), "/joyが%.2f秒以上届かないため停止しました(コントローラの接続を確認)",
          joy_timeout_sec_);
      }
      armed_ = false;
      set_enabled(false, "joy途絶");
    }

    std::lock_guard<std::mutex> lock(pub_mutex_);
    if (shutting_down_) {
      return;  // 終了処理中は有効化指令を送らない
    }
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

  // pre-shutdownコールバック(シグナル処理のスレッドから呼ばれる)
  void send_disable_on_shutdown()
  {
    {
      std::lock_guard<std::mutex> lock(pub_mutex_);
      shutting_down_ = true;
    }
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {-1.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; ++i) {
      cmd_pub_->publish(msg);
      std::this_thread::sleep_for(20ms);
    }
  }

  void set_enabled(bool enabled, const char * reason)
  {
    if (enabled != enabled_) {
      RCLCPP_INFO(get_logger(), "%s (%s)", enabled ? "有効化" : "DISABLE", reason);
    }
    enabled_ = enabled;
    if (!enabled) {
      target_vel_ = 0.0;
    }
  }

  int axis_index_{1};
  int deadman_button_index_{5};
  int estop_button_index_{9};
  double max_vel_radps_{1.0};
  double deadzone_{0.08};
  bool invert_axis_{false};
  double joy_timeout_sec_{0.5};
  int motor_number_{1};

  bool have_joy_{false};
  SteadyClock::time_point last_joy_time_{};
  bool armed_{false};    // R1を離した状態を確認済み(=次のR1押下で有効化してよい)
  bool multiple_joy_publishers_{false};  // /joyの送信元が複数(=入力を信用しない)
  bool enabled_{false};
  double target_vel_{0.0};

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::mutex pub_mutex_;       // shutting_down_と指令の送信を守る
  bool shutting_down_{false};
  rclcpp::PreShutdownCallbackHandle shutdown_cb_handle_;
};

}  // namespace robstride_ctrl

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robstride_ctrl::Ps4TeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
