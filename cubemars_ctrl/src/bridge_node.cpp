// ESP32(soki基板, MODE_CUBEMARS)とUSBシリアルで52バイトフレームを送受信するブリッジノード。
// serial_tx_<DEVICE_ID> (Int16MultiArray, 24要素) を購読してESP32へ送信し、
// ESP32からの帰還を serial_rx_<DEVICE_ID> (Int16MultiArray, 24要素) として配信する。
// トピック名・型はros2canのGUIと同じなので、GUIの代わりにこのノードを使える
// (逆に、このノードの代わりにros2canのGUIをブリッジとして使うこともできる)。
// tmotor_ctrl/src/bridge_node.cpp をベースにしている。
//
// 安全設計:
//   - serial_tx_<ID>が upstream_timeout_sec 以上届かなければ、送信内容を全スロット0
//     (=ゼロ速度・その場停止)に切り替える。ドライバノードだけが落ちた場合に、
//     このノードが最後の指令を送り続けてしまうのを防ぐため。
//   - ノード終了時に全スロット0のフレームを送信する。
//   - ESP32側にも通信途絶時のフェイルセーフ(CUBEMARS_CMD_TIMEOUT_MS)があるので、
//     このノード自体が異常終了してもモータは止まる。

#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

#include "cubemars_ctrl/frame_codec.hpp"

using namespace std::chrono_literals;

namespace cubemars_ctrl
{

class BridgeNode : public rclcpp::Node
{
public:
  BridgeNode()
  : Node("cubemars_bridge_node")
  {
    port_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    device_id_ = static_cast<uint8_t>(declare_parameter<int>("device_id", 11));
    resend_period_sec_ = declare_parameter<double>("resend_period_sec", 0.05);
    upstream_timeout_sec_ = declare_parameter<double>("upstream_timeout_sec", 0.5);

    tx_data_.fill(0);
    last_upstream_ = std::chrono::steady_clock::now();
    last_write_ = std::chrono::steady_clock::now();

    const std::string rx_topic = "serial_rx_" + std::to_string(device_id_);
    const std::string tx_topic = "serial_tx_" + std::to_string(device_id_);
    rx_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>(rx_topic, 10);
    tx_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      tx_topic, 10,
      std::bind(&BridgeNode::on_tx_command, this, std::placeholders::_1));

    open_serial();

    timer_ = create_wall_timer(5ms, std::bind(&BridgeNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(), "cubemars_bridge_node: port=%s device_id=%u (sub=%s, pub=%s)",
      port_.c_str(), device_id_, tx_topic.c_str(), rx_topic.c_str());
  }

  ~BridgeNode() override
  {
    send_zero_and_close();
  }

private:
  void open_serial()
  {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(
        get_logger(),
        "シリアルポート %s を開けません(%s)。USB接続・ポート名(ls /dev/ttyACM*)・"
        "dialoutグループ所属・ros2canのGUIやpio device monitorが掴んでいないかを確認してください。",
        port_.c_str(), std::strerror(errno));
      return;
    }

    // 他プロセス(ros2canのGUI等)からの多重オープンを防ぐ排他ロック。
    ::ioctl(fd_, TIOCEXCL);

    termios tty{};
    if (::tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattrに失敗しました: %s", std::strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return;
    }

    // XIAO ESP32-S3はネイティブUSB(CDC)なのでボーレートは実質無関係だが、
    // ファーム(Serial.begin(115200))に合わせておく。
    cfmakeraw(&tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattrに失敗しました: %s", std::strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return;
    }

    ::tcflush(fd_, TCIOFLUSH);
    parser_ = FrameParser{};
    RCLCPP_INFO(get_logger(), "シリアルポート %s をオープンしました", port_.c_str());
  }

  void on_tx_command(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    if (msg->data.size() != kSlotCount) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "serial_tx_%u の要素数が %zu です(24である必要があります)。足りない分は0で埋めます。",
        device_id_, msg->data.size());
    }
    for (size_t i = 0; i < kSlotCount; ++i) {
      tx_data_[i] = (i < msg->data.size()) ? msg->data[i] : 0;
    }
    last_upstream_ = std::chrono::steady_clock::now();
    if (upstream_timed_out_) {
      upstream_timed_out_ = false;
      RCLCPP_INFO(get_logger(), "serial_tx_%u の受信が再開しました。", device_id_);
    }
    write_tx_frame();
  }

  void write_tx_frame()
  {
    if (fd_ < 0) {
      return;
    }
    const auto frame = encode_frame(device_id_, tx_data_);
    const ssize_t written = ::write(fd_, frame.data(), frame.size());
    if (written != static_cast<ssize_t>(frame.size())) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "シリアル書き込みが不完全です(%zd/%zuバイト)", written, frame.size());
    }
    last_write_ = std::chrono::steady_clock::now();
  }

  static double seconds_since(const std::chrono::steady_clock::time_point & t)
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
  }

  void on_timer()
  {
    if (fd_ < 0) {
      // 再接続を試みる(ESP32の抜き差し・再起動に追従するため)。毎回だとログが埋まるので間引く。
      if (seconds_since(last_reconnect_attempt_) >= reconnect_interval_sec_) {
        last_reconnect_attempt_ = std::chrono::steady_clock::now();
        open_serial();
      }
      return;
    }

    read_and_publish();

    std::lock_guard<std::mutex> lock(tx_mutex_);
    if (!upstream_timed_out_ && seconds_since(last_upstream_) > upstream_timeout_sec_) {
      upstream_timed_out_ = true;
      tx_data_.fill(0);
      RCLCPP_WARN(
        get_logger(),
        "serial_tx_%u が %.1f 秒届いていません。ゼロ指令(その場停止)を送ります。",
        device_id_, upstream_timeout_sec_);
    }
    // ESP32側のフェイルセーフ(300ms)に引っかからないよう、指令が来ていなくても定期的に再送する。
    if (seconds_since(last_write_) >= resend_period_sec_) {
      write_tx_frame();
    }
  }

  void read_and_publish()
  {
    uint8_t buf[512];
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n > 0) {
      parser_.push_bytes(buf, static_cast<size_t>(n));
      while (auto frame = parser_.pop_frame()) {
        if (frame->device_id != device_id_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "DEVICE_ID=%u のフレームを受信しました(期待値 %u)。ファームのconfig.hppと"
            "config/cubemars_ctrl.yamlのdevice_idを揃えてください。",
            frame->device_id, device_id_);
          continue;
        }
        std_msgs::msg::Int16MultiArray msg;
        msg.data.assign(frame->data.begin(), frame->data.end());
        rx_pub_->publish(msg);
      }
    } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      // 注意: VMIN=0/VTIME=0の設定では「受信データが無い」ときread()は0を返すため、
      // 0は切断とみなさない。USBが抜けた場合は-1(EIO等)が返る。
      RCLCPP_WARN(
        get_logger(), "シリアル読み込みエラー(%s)。%.0f秒ごとに再接続を試みます。",
        std::strerror(errno), reconnect_interval_sec_);
      close_serial();
      last_reconnect_attempt_ = std::chrono::steady_clock::now();
    }
  }

  // 排他ロック(TIOCEXCL)はポート側に残るため、解除してから閉じる
  // (解除しないと再接続時に「Device or resource busy」で開けなくなる)。
  void close_serial()
  {
    if (fd_ >= 0) {
      ::ioctl(fd_, TIOCNXCL);
      ::close(fd_);
      fd_ = -1;
    }
  }

  void send_zero_and_close()
  {
    if (fd_ >= 0) {
      std::lock_guard<std::mutex> lock(tx_mutex_);
      tx_data_.fill(0);
      write_tx_frame();
      close_serial();
    }
  }

  std::string port_;
  uint8_t device_id_{11};
  double resend_period_sec_{0.05};
  double upstream_timeout_sec_{0.5};
  bool upstream_timed_out_{false};

  int fd_{-1};
  FrameParser parser_;
  SlotArray tx_data_{};
  std::mutex tx_mutex_;
  std::chrono::steady_clock::time_point last_upstream_{};
  std::chrono::steady_clock::time_point last_write_{};
  std::chrono::steady_clock::time_point last_reconnect_attempt_{};
  double reconnect_interval_sec_{3.0};

  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr rx_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr tx_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cubemars_ctrl

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cubemars_ctrl::BridgeNode>();
  rclcpp::spin(node);
  node.reset();  // デストラクタでゼロ指令を送ってからshutdownする
  rclcpp::shutdown();
  return 0;
}
