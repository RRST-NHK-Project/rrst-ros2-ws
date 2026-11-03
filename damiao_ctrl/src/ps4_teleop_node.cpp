// PS4コントローラ(USB有線, joyパッケージ経由)でDamiaoモータ(1〜4台)を
// テレオペするノード。モータごとに制御方法を選べる(control_modes):
//   "velocity": スティック上下(stick_y_axes)の倒し量で速度指令(VELモード)。
//   "angle":    スティック(stick_x_axes/stick_y_axes)を倒した方向をモータが向く
//               (POS_VELモード)。スティック上=基準の向き、右=基準から90°。
//               ・スティックを離す(倒し量<angle_deadzone)とその時の向きを保つ。
//               ・目標は今の目標から近い方の回り方で選ぶ(±angle_limit_radを超える時は逆回り)。
//               ・基準の向きは起動時の位置0(モータ電源投入時の向き)。△(zero_button_index)を
//                 押すと「今の向き」を基準(=スティック上)にし直す。
//               ・R1を押した瞬間の目標は現在位置(押した瞬間に急に回らない)。
//               ・回転方向が鏡写し(右に倒すと左を向く)ならinvert_axesをtrueにする。
// 2台構成の既定(config/damiao_ctrl.yaml): モータ1=左スティック上下で速度、
// モータ2=右スティックで角度。R1・Options・各種停止は全台共通。
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
//   - 角度制御のモータは、現在位置(damiao/motor<N>/state)が届いていないと有効化しない
//     (位置が分からないまま角度指令を出すと、どこへ回るか分からないため)。
//   - モータ内部の位置(stateの6番目)が分からない、または±12radを超えている場合も
//     有効化しない(速度制御で何百rad回したモータは、ゆっくり回した時に位置の読みが
//     正しく変わらず、角度指令で何百radも回ってしまうため。2026-11-02実機で確認。
//     ESP32側でも同じ条件で角度制御を拒否する)。電源を入れ直すと0に戻る。
//   - 位置の帰還は±12.5rad(P_MAX)で一周するが、モータ内部の位置は何周分も数えている
//     可能性がある。速度制御で何周も回した後に角度制御すると、内部の位置と帰還がずれて
//     いて、モータが何周も回って戻ろうとする恐れがある。そこで、有効化してからの実際の
//     移動量が、それまでに出した指令の最大量+0.5radを超えたら、そのモータを止める
//     (R1を離すまで再開しない)。角度制御に使うモータは使う前に電源を入れ直すこと
//     (電源投入時に位置0になる)。
//   - このノード自体が落ちた場合はdamiao_driver_nodeのcmd途絶の見張りで止まる。
//   - モータが故障(通信喪失等)で止まった場合は、R1を離して押し直すと
//     DISABLE->再有効化の順になり復帰する。
//
// ボタン/軸の番号(2026-10-14 実機確認: DualShock 4 CUH-ZCT1をUSB接続、joy 3.3.0、
// Linuxのplaystationドライバ。軸8本・ボタン13個):
//   buttons: 0=×, 1=○, 2=△, 3=□, 4=L1, 5=R1, 6=L2, 7=R2, 8=Share, 9=Options,
//            10=PS, 11=L3, 12=R3
//   axes:    0=左スティック左右, 1=左スティック上下(上が+1.0), 2=L2(離して+1.0),
//            3=右スティック左右, 4=右スティック上下, 5=R2(離して+1.0), 6/7=十字キー
//   左右の軸はROSの慣例で左が+1.0(右に倒すと-1.0)として扱う。
// 別のコントローラ・接続方法では番号が変わることがあるので、
// `ros2 topic echo /joy` で必ず確認すること(README参照)。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

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
    // モータごとの設定。どれも同じ長さで指定する(invert_axesは短ければ残りfalse)。
    const auto motor_numbers =
      declare_parameter<std::vector<int64_t>>("motor_numbers", std::vector<int64_t>{1});
    const auto control_modes =
      declare_parameter<std::vector<std::string>>("control_modes", std::vector<std::string>{"velocity"});
    // スティック上下の軸(1=左, 4=右)と左右の軸(0=左, 3=右)
    const auto y_axes =
      declare_parameter<std::vector<int64_t>>("stick_y_axes", std::vector<int64_t>{1});
    const auto x_axes =
      declare_parameter<std::vector<int64_t>>("stick_x_axes", std::vector<int64_t>{0});
    // velocity: 回転方向を逆にする / angle: 向きを鏡写しにする
    const auto invert_axes =
      declare_parameter<std::vector<bool>>("invert_axes", std::vector<bool>{false});
    deadman_button_index_ = declare_parameter<int>("deadman_button_index", 5);   // R1
    estop_button_index_ = declare_parameter<int>("estop_button_index", 9);       // Options
    zero_button_index_ = declare_parameter<int>("zero_button_index", 2);         // △
    max_vel_radps_ = declare_parameter<double>("max_vel_radps", 1.0);       // 初回動作確認用に控えめな値
    deadzone_ = declare_parameter<double>("deadzone", 0.08);
    angle_vel_limit_radps_ = declare_parameter<double>("angle_vel_limit_radps", 10.0);
    angle_deadzone_ = declare_parameter<double>("angle_deadzone", 0.5);
    angle_limit_rad_ = declare_parameter<double>("angle_limit_rad", 12.0);
    joy_timeout_sec_ = declare_parameter<double>("joy_timeout_sec", 0.5);

    const size_t n = motor_numbers.size();
    if (n == 0 || control_modes.size() != n || y_axes.size() != n || x_axes.size() != n) {
      RCLCPP_FATAL(
        get_logger(), "motor_numbers・control_modes・stick_y_axes・stick_x_axesは"
        "同じ長さ(1以上)にしてください");
      throw std::runtime_error("invalid teleop parameters");
    }
    if (angle_limit_rad_ < M_PI || angle_limit_rad_ > 12.4) {
      // 位置の帰還は±12.5rad(P_MAX)で一周してしまうため、その内側に収める
      RCLCPP_FATAL(get_logger(), "angle_limit_radは%.2f〜12.4の範囲にしてください", M_PI);
      throw std::runtime_error("invalid angle_limit_rad");
    }

    std::string mapping;
    for (size_t i = 0; i < n; ++i) {
      auto ch = std::make_unique<Channel>();
      ch->motor_number = static_cast<int>(motor_numbers[i]);
      if (control_modes[i] == "angle") {
        ch->angle_mode = true;
      } else if (control_modes[i] != "velocity") {
        RCLCPP_FATAL(get_logger(), "control_modesは\"velocity\"か\"angle\"です: %s",
          control_modes[i].c_str());
        throw std::runtime_error("invalid control_modes");
      }
      ch->y_axis = static_cast<int>(y_axes[i]);
      ch->x_axis = static_cast<int>(x_axes[i]);
      ch->invert = i < invert_axes.size() && invert_axes[i];
      const std::string prefix = "damiao/motor" + std::to_string(ch->motor_number);
      ch->cmd_pub = create_publisher<std_msgs::msg::Float32MultiArray>(prefix + "/cmd", 10);
      Channel * raw = ch.get();
      ch->state_sub = create_subscription<std_msgs::msg::Float32MultiArray>(
        prefix + "/state", 10,
        [raw](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
          if (msg->data.size() >= 5) {
            if (raw->active && raw->have_state) {
              // 帰還は±12.5radで一周するので、前回との差が半周(12.5rad)を超えたら補正する
              double d = msg->data[0] - raw->position;
              if (d > 12.5) {
                d -= 25.0;
              } else if (d < -12.5) {
                d += 25.0;
              }
              raw->moved += d;
            }
            raw->position = msg->data[0];
            raw->internal_pos = msg->data.size() >= 6 ? msg->data[5] : std::nan("");
            raw->state_ok = msg->data[4] > 0.5f;
            raw->last_state_time = SteadyClock::now();
            raw->have_state = true;
          }
        });
      mapping += " モータ" + std::to_string(ch->motor_number) +
        (ch->angle_mode ?
        "=角度(軸" + std::to_string(ch->x_axis) + "," + std::to_string(ch->y_axis) + ")" :
        "=速度(軸" + std::to_string(ch->y_axis) + ")") + (ch->invert ? "反転" : "");
      channels_.push_back(std::move(ch));
    }

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Ps4TeleopNode::on_joy, this, std::placeholders::_1));

    // 50Hzで最新の指令を送る(damiao_driver_nodeのcmd途絶の見張りより十分速い)。
    publish_timer_ = create_wall_timer(20ms, std::bind(&Ps4TeleopNode::publish_cmd, this));

    // 終了時(Ctrl+C)はROSの通信を切る前にDISABLEを送る(damiao_driver_nodeと同じ理由で
    // rclcpp::on_shutdownは使わない)。
    shutdown_cb_handle_ = get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      std::bind(&Ps4TeleopNode::send_disable_on_shutdown, this));

    RCLCPP_INFO(
      get_logger(),
      "ps4_teleop_node:%s deadman=%d estop=%d zero=%d max_vel=%.2frad/s "
      "angle_vel_limit=%.2frad/s joy_timeout=%.2fs (R1を押している間だけ動く。"
      "実際のボタン配置は `ros2 topic echo /joy` で要確認)",
      mapping.c_str(), deadman_button_index_, estop_button_index_, zero_button_index_,
      max_vel_radps_, angle_vel_limit_radps_, joy_timeout_sec_);
  }

  ~Ps4TeleopNode() override
  {
    get_node_base_interface()->get_context()->remove_pre_shutdown_callback(shutdown_cb_handle_);
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  // モータ1台分の割り当て
  struct Channel
  {
    int motor_number{1};
    bool angle_mode{false};
    int y_axis{1};
    int x_axis{0};
    bool invert{false};
    double target_vel{0.0};    // velocity: 速度指令[rad/s]
    double target_pos{0.0};    // angle: 角度指令[rad](モータの位置の値)
    double zero_pos{0.0};      // angle: スティック上に対応するモータの位置[rad]
    bool active{false};        // このモータに有効化指令を出しているか
    // angle: 位置ずれの見張り
    double start_pos{0.0};     // 有効化した時の位置
    double moved{0.0};         // 有効化してからの実際の移動量(帰還の一周を補正して積算)
    double max_offset{0.0};    // 有効化してから出した指令の、start_posからの最大の離れ
    bool blocked{false};       // 位置ずれで止めた(R1を離すまで再開しない)
    // damiao/motor<N>/state から
    double position{0.0};
    double internal_pos{std::nan("")};  // モータ内部の位置(不明ならNaN)
    bool state_ok{false};
    bool have_state{false};
    SteadyClock::time_point last_state_time{};
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr state_sub;
  };

  bool position_known(const Channel & ch) const
  {
    return ch.have_state && ch.state_ok &&
           std::chrono::duration<double>(SteadyClock::now() - ch.last_state_time).count() < 0.3;
  }

  static bool axis_in(const sensor_msgs::msg::Joy & msg, int index)
  {
    return index >= 0 && static_cast<size_t>(index) < msg.axes.size();
  }

  // スティックの向き[rad](上=0、右=+90°、時計回りが+)を、今の目標に近い回り方の
  // 目標角度へ変換する。±angle_limit_radを超える時は反対回りの候補を使う。
  double nearest_angle_target(const Channel & ch, double stick_angle) const
  {
    const double desired = ch.zero_pos + (ch.invert ? -stick_angle : stick_angle);
    const double k = std::round((ch.target_pos - desired) / (2.0 * M_PI));
    double best = desired + 2.0 * M_PI * k;
    if (best > angle_limit_rad_) {
      best -= 2.0 * M_PI;
    } else if (best < -angle_limit_rad_) {
      best += 2.0 * M_PI;
    }
    return best;
  }

  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_time_ = SteadyClock::now();
    have_joy_ = true;
    if (multiple_joy_publishers_) {
      return;  // どの/joyが本物か分からないので、R1の状態も含めて一切使わない
    }

    bool axis_ok = true;
    for (const auto & ch : channels_) {
      axis_ok = axis_ok && axis_in(*msg, ch->y_axis) && (!ch->angle_mode || axis_in(*msg, ch->x_axis));
    }
    const auto button_ok = [&msg](int index) {
        return index >= 0 && static_cast<size_t>(index) < msg->buttons.size();
      };
    if (!axis_ok || !button_ok(deadman_button_index_) || !button_ok(estop_button_index_)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "/joyの軸/ボタン数(axes=%zu, buttons=%zu)が設定した番号に足りません。"
        "config/damiao_ctrl.yamlを確認してください", msg->axes.size(), msg->buttons.size());
      set_enabled(false, "設定エラー");
      armed_ = false;
      return;
    }

    // △: 角度制御のモータの「今の向き」をスティック上(基準)にする
    const bool zero_pressed = button_ok(zero_button_index_) && msg->buttons[zero_button_index_] != 0;
    if (zero_pressed && !zero_was_pressed_) {
      for (auto & ch : channels_) {
        if (!ch->angle_mode) {
          continue;
        }
        if (position_known(*ch)) {
          ch->zero_pos = ch->position;
          ch->target_pos = ch->position;  // 押した瞬間に回り出さないよう、目標も今の位置にする
          // 位置ずれの見張りも、ここを起点に数え直す
          ch->start_pos = ch->position;
          ch->moved = 0.0;
          ch->max_offset = 0.0;
          RCLCPP_INFO(
            get_logger(), "モータ%d: 今の向き(%.3frad)をスティック上(基準)にしました",
            ch->motor_number, ch->zero_pos);
        } else {
          RCLCPP_WARN(get_logger(), "モータ%d: 位置が分からないので基準を設定できません",
            ch->motor_number);
        }
      }
    }
    zero_was_pressed_ = zero_pressed;

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

    set_enabled(true, "R1押下");
    for (auto & ch : channels_) {
      const double y = static_cast<double>(msg->axes[ch->y_axis]);
      if (!ch->angle_mode) {
        const double raw = std::abs(y) < deadzone_ ? 0.0 : y;
        ch->target_vel = (ch->invert ? -raw : raw) * max_vel_radps_;
        ch->active = true;
        continue;
      }
      if (ch->blocked) {
        continue;  // 位置ずれで止めた。R1を離すまで再開しない
      }
      if (!ch->active) {
        // 有効化した瞬間: 位置が分かっていれば、目標=今の位置から始める
        if (!position_known(*ch)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "モータ%d: 現在位置が届いていないので角度制御を有効化しません", ch->motor_number);
          continue;
        }
        if (std::isnan(ch->internal_pos) || std::abs(ch->internal_pos) > 12.0) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "モータ%d: 内部の位置が%sのため角度制御を有効化しません"
            "(速度制御でたくさん回した後は、モータの電源を入れ直すと0に戻ります)", ch->motor_number,
            std::isnan(ch->internal_pos) ? "不明" :
            (std::to_string(static_cast<int>(std::round(ch->internal_pos))) + "rad(±12radを超えている)").c_str());
          continue;
        }
        ch->target_pos = ch->position;
        ch->start_pos = ch->position;
        ch->moved = 0.0;
        ch->max_offset = 0.0;
        ch->active = true;
      }
      // 位置ずれの見張り: 出した指令より明らかに大きく回っていたら止める
      if (std::abs(ch->moved) > ch->max_offset + 0.5) {
        ch->blocked = true;
        ch->active = false;
        RCLCPP_ERROR(
          get_logger(), "モータ%d: 指令(最大%.2frad)より大きく%.2frad回ったため止めました。"
          "位置の帰還とモータ内部の位置がずれています(速度制御で何周も回した後などに起きる)。"
          "モータ%dの電源を入れ直してから、R1を押し直してください",
          ch->motor_number, ch->max_offset, ch->moved, ch->motor_number);
        continue;
      }
      // 左右の軸は左が+なので、右を+にしてから向きを求める(上=0、右=+90°)
      const double x_right = -static_cast<double>(msg->axes[ch->x_axis]);
      if (std::hypot(x_right, y) >= angle_deadzone_) {
        ch->target_pos = nearest_angle_target(*ch, std::atan2(x_right, y));
      }
      ch->max_offset = std::max(ch->max_offset, std::abs(ch->target_pos - ch->start_pos));
    }
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
    for (const auto & ch : channels_) {
      std_msgs::msg::Float32MultiArray msg;
      if (enabled_ && ch->active && ch->angle_mode) {
        // control_mode=1.0(POS_VEL), target=角度[rad], vel_limit=回る速さの上限[rad/s]
        msg.data = {1.0f, static_cast<float>(ch->target_pos),
          static_cast<float>(angle_vel_limit_radps_)};
      } else if (enabled_ && ch->active) {
        // control_mode=0.0(VEL), target=target_vel[rad/s], vel_limit未使用
        msg.data = {0.0f, static_cast<float>(ch->target_vel), 0.0f};
      } else {
        // control_mode=-1.0(DISABLE)
        msg.data = {-1.0f, 0.0f, 0.0f};
      }
      ch->cmd_pub->publish(msg);
    }
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
      for (const auto & ch : channels_) {
        ch->cmd_pub->publish(msg);
      }
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
      for (auto & ch : channels_) {
        ch->target_vel = 0.0;
        ch->active = false;  // 次に有効化する時、角度の目標は改めて現在位置から始める
        ch->blocked = false;
      }
    }
  }

  int deadman_button_index_{5};
  int estop_button_index_{9};
  int zero_button_index_{2};
  double max_vel_radps_{1.0};
  double deadzone_{0.08};
  double angle_vel_limit_radps_{10.0};
  double angle_deadzone_{0.5};
  double angle_limit_rad_{12.0};
  double joy_timeout_sec_{0.5};
  // unique_ptrで持つ(stateのコールバックがChannelのアドレスを保持するため)
  std::vector<std::unique_ptr<Channel>> channels_;

  bool have_joy_{false};
  SteadyClock::time_point last_joy_time_{};
  bool armed_{false};    // R1を離した状態を確認済み(=次のR1押下で有効化してよい)
  bool multiple_joy_publishers_{false};  // /joyの送信元が複数(=入力を信用しない)
  bool zero_was_pressed_{false};
  bool enabled_{false};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::mutex pub_mutex_;       // shutting_down_と指令の送信を守る
  bool shutting_down_{false};
  rclcpp::PreShutdownCallbackHandle shutdown_cb_handle_;
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
