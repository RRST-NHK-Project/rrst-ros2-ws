// =====================================================================
// example_motor_control : 自分のプログラムからモータを動かす「お手本」
// =====================================================================
// ■ 動き(全部で約11秒。終わったら2台とも脱力して終了する)
//    0〜 3秒  モータ1を速度 2rad/s で回す(速度制御)
//    3〜 4秒  モータ1を速度0にして止める
//    4〜 7秒  モータ2を「始めた時の向き + 90°」へ向ける(角度制御、速さ3rad/s)
//    7〜10秒  モータ2を「始めた時の向き」へ戻す
//   10秒〜    2台とも脱力して終了
//
// ■ 使い方(ros2can と damiao_driver_node を先に起動しておく。開発ガイド参照)
//   ros2 run damiao_ctrl example_motor_control
//   実機が無い時は scripts/fake_esp32.py で、ESP32とモータのふりをさせて試せる。
//
// ■ 自分の動きにするには
//   run_step() の中の「ここを書き換える」部分だけを変えればよい。
//   使える命令(MotorHandleクラス):
//     set_velocity(速度rad/s)            速度制御
//     set_angle(角度rad, 速さrad/s)      角度制御(その角度へ、速さ以下で回る)
//     set_disable()                      脱力
//     position() / velocity() / torque() 今の位置[rad]・速度[rad/s]・トルク[N・m]
//
// ■ 安全のための決まり(このお手本は全部守っている)
//   ・指令は50回/秒で送り続ける(0.5秒届かないと駆動ノードが自動で脱力させる)。
//   ・始める前に、モータから返事が届いていることを確かめる。
//   ・角度制御は、モータ内部の位置が±12rad以内の時だけ使う(超えていたら電源を入れ直す)。
//   ・終わる時・Ctrl+Cの時は、必ず脱力を送る。
// =====================================================================

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

namespace damiao_ctrl
{

// ---------------------------------------------------------------------
// モータ1台分の「リモコン」。damiao/motorN/cmd に指令を送り、
// damiao/motorN/state から今の状態を受け取る。
// ---------------------------------------------------------------------
class MotorHandle
{
public:
  MotorHandle(rclcpp::Node & node, int number)
  : number_(number)
  {
    const std::string prefix = "damiao/motor" + std::to_string(number);
    cmd_pub_ = node.create_publisher<std_msgs::msg::Float32MultiArray>(prefix + "/cmd", 10);
    state_sub_ = node.create_subscription<std_msgs::msg::Float32MultiArray>(
      prefix + "/state", 10,
      [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 6) {
          position_ = msg->data[0];
          velocity_ = msg->data[1];
          torque_ = msg->data[2];
          state_ = static_cast<int>(msg->data[3]);
          feedback_ok_ = msg->data[4] > 0.5f;
          internal_pos_ = msg->data[5];
          last_state_time_ = std::chrono::steady_clock::now();
        }
      });
  }

  // ---- 指令を決める(実際に送るのは send()) ----
  void set_velocity(double radps) {cmd_ = {0.0f, static_cast<float>(radps), 0.0f};}
  void set_angle(double rad, double speed_radps)
  {
    cmd_ = {1.0f, static_cast<float>(rad), static_cast<float>(speed_radps)};
  }
  void set_disable() {cmd_ = {-1.0f, 0.0f, 0.0f};}

  // 今の指令を送る。タイマーから50回/秒で呼ぶ。
  void send()
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {cmd_[0], cmd_[1], cmd_[2]};
    cmd_pub_->publish(msg);
  }

  // ---- 今の状態 ----
  double position() const {return position_;}   // 位置[rad](±12.5radで一周)
  double velocity() const {return velocity_;}   // 速度[rad/s]
  double torque() const {return torque_;}       // トルク[N・m]
  int state() const {return state_;}            // 0=脱力 1=有効 それ以外=故障
  // モータから返事が届いているか(0.3秒以内に届いていて、帰還OK)
  bool ready() const
  {
    const auto age = std::chrono::steady_clock::now() - last_state_time_;
    return feedback_ok_ && age < 300ms;
  }
  // 角度制御を使ってよいか(モータ内部の位置が分かっていて±12rad以内)
  bool angle_ok() const {return !std::isnan(internal_pos_) && std::abs(internal_pos_) <= 12.0;}
  int number() const {return number_;}

private:
  int number_;
  std::array<float, 3> cmd_{-1.0f, 0.0f, 0.0f};  // 最初は脱力
  double position_{0.0};
  double velocity_{0.0};
  double torque_{0.0};
  int state_{0};
  bool feedback_ok_{false};
  double internal_pos_{std::nan("")};
  std::chrono::steady_clock::time_point last_state_time_{};
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr state_sub_;
};

// ---------------------------------------------------------------------
// お手本のノード本体
// ---------------------------------------------------------------------
class ExampleMotorControl : public rclcpp::Node
{
public:
  ExampleMotorControl()
  : Node("example_motor_control"), m1_(*this, 1), m2_(*this, 2)
  {
    // 50回/秒(20msごと)に on_timer() を呼ぶ
    timer_ = create_wall_timer(20ms, std::bind(&ExampleMotorControl::on_timer, this));
    // Ctrl+Cで止めた時も、通信が切れる前に必ず脱力を送る
    shutdown_cb_ = get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      [this]() {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        finished_ = true;
        for (int i = 0; i < 3; ++i) {
          m1_.set_disable(); m2_.set_disable(); m1_.send(); m2_.send();
          std::this_thread::sleep_for(20ms);
        }
      });
    created_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "お手本を開始します。モータから返事が届くのを待っています…");
  }

  ~ExampleMotorControl() override
  {
    get_node_base_interface()->get_context()->remove_pre_shutdown_callback(shutdown_cb_);
  }

private:
  // ===================================================================
  // 【ここを書き換える】t = 動き始めてからの秒数。tに合わせて指令を決める。
  // ===================================================================
  void run_step(double t)
  {
    if (t < 3.0) {
      m1_.set_velocity(2.0);       // モータ1: 速度 2rad/s
      m2_.set_disable();           // モータ2: 脱力のまま
    } else if (t < 4.0) {
      m1_.set_velocity(0.0);       // モータ1: 速度0(止める)
    } else if (t < 7.0) {
      m1_.set_disable();
      if (use_angle_) {
        m2_.set_angle(start_angle2_ + quarter_turn_, 3.0);  // モータ2: +90°へ、3rad/sで
      }
    } else if (t < 10.0) {
      if (use_angle_) {
        m2_.set_angle(start_angle2_, 3.0);                   // モータ2: 元の向きへ
      }
    } else {
      finish();                    // 2台とも脱力して終わる
    }
  }
  // ===================================================================

  void on_timer()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (finished_) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();

    if (!started_) {
      // 始める前の確認: 2台とも返事が届くまで待つ(5秒待っても来なければやめる)
      if (m1_.ready() && m2_.ready()) {
        start_angle2_ = m2_.position();
        use_angle_ = m2_.angle_ok();
        // +90°が位置の範囲(±12rad)を超えるなら、反対の-90°にする
        quarter_turn_ = (start_angle2_ + M_PI / 2 > 12.0) ? -M_PI / 2 : M_PI / 2;
        if (!use_angle_) {
          RCLCPP_WARN(get_logger(), "モータ2の内部の位置が±12radを超えている(または不明)ので、"
            "角度制御は飛ばします(モータの電源を入れ直すと0に戻ります)");
        }
        started_ = true;
        start_time_ = now;
        RCLCPP_INFO(get_logger(), "動き始めます(モータ2の始めの向き %+.3frad)", start_angle2_);
      } else if (now - created_ > 5s) {
        RCLCPP_ERROR(get_logger(), "5秒待ってもモータから返事が来ません"
          "(ros2can・damiao_driver_nodeの起動、モータの電源、ROS_AUTOMATIC_DISCOVERY_RANGEを確認)");
        finish();
        return;
      }
      m1_.send(); m2_.send();  // 待っている間も脱力を送り続ける
      return;
    }

    // 故障したら全部止める
    if (m1_.state() > 1 || m2_.state() > 1 || !m1_.ready() || !m2_.ready()) {
      RCLCPP_ERROR(get_logger(), "故障または返事の途絶を検出したので止めます");
      finish();
      return;
    }

    const double t = std::chrono::duration<double>(now - start_time_).count();
    run_step(t);
    if (finished_) {
      return;
    }
    m1_.send(); m2_.send();

    // 1秒ごとに様子を表示
    if (t - last_print_ >= 1.0) {
      last_print_ = t;
      RCLCPP_INFO(
        get_logger(), "t=%4.1fs  モータ1: 速度%+6.2frad/s  モータ2: 位置%+6.3frad",
        t, m1_.velocity(), m2_.position());
    }
  }

  // 2台とも脱力を送ってからノードを終える
  void finish()
  {
    finished_ = true;
    m1_.set_disable(); m2_.set_disable();
    for (int i = 0; i < 5; ++i) {
      m1_.send(); m2_.send();
      std::this_thread::sleep_for(20ms);
    }
    RCLCPP_INFO(get_logger(), "終わりました(2台とも脱力)");
    rclcpp::shutdown();
  }

  MotorHandle m1_;
  MotorHandle m2_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::PreShutdownCallbackHandle shutdown_cb_;
  // タイマーとCtrl+C時の処理が同時に指令を送らないようにする鍵。finish()の中の
  // rclcpp::shutdown()が同じスレッドで終了処理(鍵を取る)を呼ぶので、同じスレッドなら
  // 何度でも取れるrecursive_mutexにしている(普通のmutexだと固まる)。
  std::recursive_mutex mutex_;
  bool started_{false};
  bool finished_{false};
  bool use_angle_{false};
  double start_angle2_{0.0};
  double quarter_turn_{M_PI / 2};
  double last_print_{-1.0};
  std::chrono::steady_clock::time_point created_;
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace damiao_ctrl

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<damiao_ctrl::ExampleMotorControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
