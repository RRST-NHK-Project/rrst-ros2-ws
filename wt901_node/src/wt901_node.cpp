// Copyright 2024 RRST NHK Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class WT901CNode : public rclcpp::Node
{
public:
  WT901CNode()
  : Node("wt901c_node"), serial_fd_(-1), running_(false)
  {
    declare_parameter<std::string>("port", "/dev/ttyUSB0");
    declare_parameter<int>("baudrate", 115200);
    declare_parameter<std::string>("frame_id", "imu_link");

    port_ = get_parameter("port").as_string();
    baudrate_ = get_parameter("baudrate").as_int();
    frame_id_ = get_parameter("frame_id").as_string();

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

    running_ = true;
    read_thread_ = std::thread(&WT901CNode::read_loop, this);
  }

  ~WT901CNode()
  {
    running_ = false;
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    close_serial();
  }

private:
  // WT901C packet structure:
  //   buf[0]     : 0x55  (header)
  //   buf[1]     : type  (0x51=accel, 0x52=gyro, 0x53=euler)
  //   buf[2..9]  : three int16 values (little-endian)
  //   buf[10]    : checksum = (sum of buf[0..9]) & 0xFF
  static constexpr size_t PACKET_SIZE = 11;
  static constexpr uint8_t HEADER = 0x55;

  enum class ReadResult { OK, CHECKSUM_ERROR, IO_ERROR, ABORTED };

  int serial_fd_;
  std::string port_;
  int baudrate_;
  std::string frame_id_;
  std::atomic<bool> running_;
  std::thread read_thread_;

  float ax_ = 0.0f, ay_ = 0.0f, az_ = 0.0f;
  float gx_ = 0.0f, gy_ = 0.0f, gz_ = 0.0f;
  float roll_ = 0.0f, pitch_ = 0.0f, yaw_ = 0.0f;

  speed_t to_baud_constant(int baud)
  {
    switch (baud) {
      case 9600:   return B9600;
      case 19200:  return B19200;
      case 38400:  return B38400;
      case 57600:  return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      default:
        RCLCPP_WARN(get_logger(), "Unsupported baud rate %d, defaulting to 115200", baud);
        return B115200;
    }
  }

  bool open_serial()
  {
    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(
        get_logger(), "Failed to open serial port %s: %s",
        port_.c_str(), strerror(errno));
      return false;
    }

    struct termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    // Raw mode: no line processing, no echo, no special characters
    cfmakeraw(&tty);

    speed_t baud = to_baud_constant(baudrate_);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    // 8N1, no flow control
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= (CLOCAL | CREAD);

    // VMIN=0, VTIME=1: return when data available, timeout 100ms
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    tcflush(serial_fd_, TCIFLUSH);
    RCLCPP_INFO(get_logger(), "Opened %s at %d baud", port_.c_str(), baudrate_);
    return true;
  }

  void close_serial()
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  // Read exactly `n` bytes into `buf`, looping on partial reads and timeouts.
  // Returns false on I/O error; sets *io_error = true for hard errors.
  bool read_exact(uint8_t * buf, size_t n, bool * io_error)
  {
    size_t total = 0;
    while (total < n && running_) {
      ssize_t ret = read(serial_fd_, buf + total, n - total);
      if (ret > 0) {
        total += static_cast<size_t>(ret);
      } else if (ret == 0) {
        // Timeout (VMIN=0, VTIME>0) — no data yet, keep waiting
      } else {
        RCLCPP_DEBUG(get_logger(), "read() error: %s", strerror(errno));
        *io_error = true;
        return false;
      }
    }
    return running_;
  }

  // Reads one complete, verified packet into buf[PACKET_SIZE].
  ReadResult read_packet(uint8_t buf[PACKET_SIZE])
  {
    // Scan byte-by-byte until we find the header 0x55
    bool io_err = false;
    while (running_) {
      if (!read_exact(buf, 1, &io_err)) {
        return io_err ? ReadResult::IO_ERROR : ReadResult::ABORTED;
      }
      if (buf[0] == HEADER) {
        break;
      }
    }
    if (!running_) {
      return ReadResult::ABORTED;
    }

    // Read remaining 10 bytes of the packet
    if (!read_exact(buf + 1, PACKET_SIZE - 1, &io_err)) {
      return io_err ? ReadResult::IO_ERROR : ReadResult::ABORTED;
    }

    // Verify checksum: sum of bytes 0..9 should equal byte 10
    uint8_t sum = 0;
    for (size_t i = 0; i < PACKET_SIZE - 1; ++i) {
      sum += buf[i];
    }
    if (sum != buf[PACKET_SIZE - 1]) {
      RCLCPP_DEBUG(
        get_logger(),
        "Checksum mismatch: computed 0x%02X, received 0x%02X",
        static_cast<unsigned>(sum),
        static_cast<unsigned>(buf[PACKET_SIZE - 1]));
      return ReadResult::CHECKSUM_ERROR;
    }

    return ReadResult::OK;
  }

  void parse_and_publish(const uint8_t buf[PACKET_SIZE])
  {
    int16_t d1 = static_cast<int16_t>((buf[3] << 8) | buf[2]);
    int16_t d2 = static_cast<int16_t>((buf[5] << 8) | buf[4]);
    int16_t d3 = static_cast<int16_t>((buf[7] << 8) | buf[6]);

    switch (buf[1]) {
      case 0x51:  // Acceleration (range ±16g)
        ax_ = d1 / 32768.0f * 16.0f * 9.8f;
        ay_ = d2 / 32768.0f * 16.0f * 9.8f;
        az_ = d3 / 32768.0f * 16.0f * 9.8f;
        break;
      case 0x52:  // Angular velocity (range ±2000 °/s)
        gx_ = d1 / 32768.0f * 2000.0f * static_cast<float>(M_PI) / 180.0f;
        gy_ = d2 / 32768.0f * 2000.0f * static_cast<float>(M_PI) / 180.0f;
        gz_ = d3 / 32768.0f * 2000.0f * static_cast<float>(M_PI) / 180.0f;
        break;
      case 0x53:  // Euler angles (range ±180°)
        roll_  = d1 / 32768.0f * static_cast<float>(M_PI);
        pitch_ = d2 / 32768.0f * static_cast<float>(M_PI);
        yaw_   = d3 / 32768.0f * static_cast<float>(M_PI);
        publish_imu();
        break;
      default:
        break;
    }
  }

  void read_loop()
  {
    while (running_) {
      if (serial_fd_ < 0) {
        if (!open_serial()) {
          std::this_thread::sleep_for(1s);
          continue;
        }
      }

      uint8_t buf[PACKET_SIZE];
      ReadResult result = read_packet(buf);

      switch (result) {
        case ReadResult::OK:
          parse_and_publish(buf);
          break;
        case ReadResult::CHECKSUM_ERROR:
          // Packet corrupted — skip it and re-sync on the next 0x55
          break;
        case ReadResult::IO_ERROR:
          RCLCPP_WARN(get_logger(), "Serial I/O error, attempting reconnect...");
          close_serial();
          std::this_thread::sleep_for(500ms);
          break;
        case ReadResult::ABORTED:
          break;
      }
    }
  }

  void publish_imu()
  {
    auto msg = sensor_msgs::msg::Imu();

    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    msg.linear_acceleration.x = ax_;
    msg.linear_acceleration.y = ay_;
    msg.linear_acceleration.z = az_;

    msg.angular_velocity.x = gx_;
    msg.angular_velocity.y = gy_;
    msg.angular_velocity.z = gz_;

    // ZYX Euler angles to quaternion
    double cr = std::cos(roll_ / 2.0);
    double sr = std::sin(roll_ / 2.0);
    double cp = std::cos(pitch_ / 2.0);
    double sp = std::sin(pitch_ / 2.0);
    double cy = std::cos(yaw_ / 2.0);
    double sy = std::sin(yaw_ / 2.0);

    msg.orientation.w = cr * cp * cy + sr * sp * sy;
    msg.orientation.x = sr * cp * cy - cr * sp * sy;
    msg.orientation.y = cr * sp * cy + sr * cp * sy;
    msg.orientation.z = cr * cp * sy - sr * sp * cy;

    // Mark covariances as unknown (-1 in first element)
    msg.orientation_covariance[0] = -1.0;
    msg.angular_velocity_covariance[0] = -1.0;
    msg.linear_acceleration_covariance[0] = -1.0;

    imu_pub_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WT901CNode>());
  rclcpp::shutdown();
  return 0;
}