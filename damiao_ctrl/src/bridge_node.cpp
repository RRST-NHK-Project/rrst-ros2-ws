// ESP32とUSBシリアルで52バイトフレームを送受信するブリッジノード。
// serial_tx_<DEVICE_ID> (Int16MultiArray, 24要素) を購読して即時送信し、
// serial_rx_<DEVICE_ID> (Int16MultiArray, 24要素) としてESP32からの
// フィードバックを配信する。ros2canのserial_tx/rx_[ID]と同じ命名・型に
// 合わせてあるため、将来ros2can側へ統合する場合も上位ノードの変更は不要。
//
// 安全設計: ノード終了時(rclcpp::on_shutdown)に全スロット0のフレームを
// 送信する。ESP32ファームウェア側には通信途絶時のフェイルセーフが無い
// (最後に受信した指令を保持し続ける)前提のため、ここでのゼロ送信が
// 実質的な安全停止の役割を持つ。

#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

#include "damiao_ctrl/frame_codec.hpp"

using namespace std::chrono_literals;

namespace damiao_ctrl
{

class BridgeNode : public rclcpp::Node
{
public:
  BridgeNode()
  : Node("damiao_bridge_node")
  {
    port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    baud_ = declare_parameter<int>("baud_rate", 115200);
    device_id_ = static_cast<uint8_t>(declare_parameter<int>("device_id", 1));
    resend_divider_ = declare_parameter<int>("resend_divider", 2);  // 10ms*2=20ms(50Hz)で再送

    tx_data_.fill(0);

    const std::string rx_topic = "serial_rx_" + std::to_string(device_id_);
    const std::string tx_topic = "serial_tx_" + std::to_string(device_id_);
    rx_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>(rx_topic, 10);
    tx_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      tx_topic, 10,
      std::bind(&BridgeNode::on_tx_command, this, std::placeholders::_1));

    open_serial();

    timer_ = create_wall_timer(10ms, std::bind(&BridgeNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(), "damiao_bridge_node: port=%s baud=%d device_id=%u (sub=%s, pub=%s)",
      port_.c_str(), baud_, device_id_, tx_topic.c_str(), rx_topic.c_str());
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
        get_logger(), "シリアルポート %s を開けません。配線・dialoutグループ所属・ポート名を確認してください。",
        port_.c_str());
      return;
    }

    // 他プロセスからの多重オープンを防ぐ排他ロック(ros2canと同じ二重排他の考え方)。
    ::ioctl(fd_, TIOCEXCL);

    termios tty{};
    if (::tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattrに失敗しました: %s", std::strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return;
    }

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
    for (size_t i = 0; i < kSlotCount; ++i) {
      tx_data_[i] = (i < msg->data.size()) ? msg->data[i] : 0;
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
      RCLCPP_WARN(get_logger(), "シリアル書き込みが不完全です(%zd/%zuバイト)", written, frame.size());
    }
  }

  void on_timer()
  {
    if (fd_ < 0) {
      // 再接続を試みる(ESP32の抜き差し・再起動に追従するため)。
      // 10ms周期のタイマーで毎回開こうとするとログが埋まるので、
      // reconnect_interval_sec_間隔に間引く(rclcpp::Timeはクロック種別が
      // 混在すると例外を投げるためsteady_clockで単純に測る)。
      const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_reconnect_attempt_).count();
      if (elapsed >= reconnect_interval_sec_) {
        last_reconnect_attempt_ = std::chrono::steady_clock::now();
        open_serial();
      }
      return;
    }

    read_and_publish();

    if (++tick_ % resend_divider_ == 0) {
      std::lock_guard<std::mutex> lock(tx_mutex_);
      write_tx_frame();
    }
  }

  void read_and_publish()
  {
    uint8_t buf[256];
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n > 0) {
      parser_.push_bytes(buf, static_cast<size_t>(n));
      while (auto frame = parser_.pop_frame()) {
        if (frame->device_id != device_id_) {
          continue;  // 自分宛でないフレームは無視
        }
        std_msgs::msg::Int16MultiArray msg;
        msg.data.assign(frame->data.begin(), frame->data.end());
        rx_pub_->publish(msg);
        last_rx_time_ = now();
      }
    } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN(get_logger(), "シリアル読み込みエラー: %s。再接続します。", std::strerror(errno));
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
      ::ioctl(fd_, TIOCNXCL);
      ::close(fd_);
      fd_ = -1;
    }
  }

  std::string port_;
  int baud_{115200};
  uint8_t device_id_{1};
  int resend_divider_{2};
  uint64_t tick_{0};

  int fd_{-1};
  FrameParser parser_;
  SlotArray tx_data_{};
  std::mutex tx_mutex_;
  rclcpp::Time last_rx_time_;
  std::chrono::steady_clock::time_point last_reconnect_attempt_{};
  double reconnect_interval_sec_{3.0};

  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr rx_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr tx_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace damiao_ctrl

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<damiao_ctrl::BridgeNode>();
  rclcpp::on_shutdown([node]() {});  // デストラクタでゼロ送信するのでここでは何もしない
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
