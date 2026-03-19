#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

static constexpr size_t PACKET_SIZE = 11;
static constexpr uint8_t PACKET_HEADER = 0x55;

class WT901CNode : public rclcpp::Node {
public:
    WT901CNode() : Node("wt901c_node"), serial_fd_(-1) {
        declare_parameter<std::string>("port", "/dev/ttyUSB0");
        port_ = get_parameter("port").as_string();

        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

        open_serial();

        // Poll for incoming bytes at 5 ms intervals
        timer_ = create_wall_timer(5ms, std::bind(&WT901CNode::read_serial, this));

        // Attempt reconnection every 1 s if the port is not open
        reconnect_timer_ = create_wall_timer(1000ms, std::bind(&WT901CNode::try_reconnect, this));
    }

    ~WT901CNode() {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    int serial_fd_;
    std::string port_;
    std::deque<uint8_t> rx_buf_;

    bool open_serial() {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            serial_fd_ = -1;
        }
        rx_buf_.clear();

        serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to open %s: %s", port_.c_str(), strerror(errno));
            return false;
        }

        struct termios tty{};
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        // Raw mode: no processing of input/output bytes
        cfmakeraw(&tty);

        // 115200 baud, 8N1, no flow control
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);
        tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
        tty.c_cflag |= CS8 | CLOCAL | CREAD;

        // Non-blocking: return immediately with whatever bytes are available
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        tcflush(serial_fd_, TCIOFLUSH);
        RCLCPP_INFO(get_logger(), "Opened serial port: %s", port_.c_str());
        return true;
    }

    void try_reconnect() {
        if (serial_fd_ < 0) {
            RCLCPP_INFO(get_logger(), "Attempting to reconnect to %s", port_.c_str());
            open_serial();
        }
    }

    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

    void read_serial() {
        if (serial_fd_ < 0) {
            return;
        }

        // Drain all available bytes into the ring buffer
        uint8_t tmp[256];
        ssize_t n = read(serial_fd_, tmp, sizeof(tmp));
        if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_WARN(get_logger(), "Serial read error (%s), closing port", strerror(errno));
                close(serial_fd_);
                serial_fd_ = -1;
                rx_buf_.clear();
            }
            return;
        }
        if (n > 0) {
            rx_buf_.insert(rx_buf_.end(), tmp, tmp + n);
        }

        // Process every complete packet in the buffer
        while (rx_buf_.size() >= PACKET_SIZE) {
            // Advance to the next header byte — O(1) pop_front per discarded byte
            while (!rx_buf_.empty() && rx_buf_.front() != PACKET_HEADER) {
                rx_buf_.pop_front();
            }
            if (rx_buf_.size() < PACKET_SIZE) {
                break;
            }

            // Verify checksum: sum of bytes [0..9] must equal byte [10]
            uint8_t checksum = 0;
            for (size_t i = 0; i < PACKET_SIZE - 1; ++i) {
                checksum += rx_buf_[i];
            }
            if (checksum != rx_buf_[PACKET_SIZE - 1]) {
                // Bad packet — skip this header byte and try to re-sync
                rx_buf_.pop_front();
                continue;
            }

            // Copy packet to contiguous storage for processing
            uint8_t pkt[PACKET_SIZE];
            for (size_t i = 0; i < PACKET_SIZE; ++i) {
                pkt[i] = rx_buf_[i];
            }
            for (size_t i = 0; i < PACKET_SIZE; ++i) {
                rx_buf_.pop_front();
            }
            process_packet(pkt);
        }

        // Guard against unbounded buffer growth
        while (rx_buf_.size() > 256) {
            rx_buf_.pop_front();
        }
    }

    void process_packet(const uint8_t *pkt) {
        // Read a little-endian signed 16-bit integer from the packet at byte offset lo
        auto read_int16 = [&](size_t lo) -> int16_t {
            return static_cast<int16_t>((static_cast<uint16_t>(pkt[lo + 1]) << 8) | pkt[lo]);
        };

        if (pkt[1] == 0x51) {
            ax = read_int16(2) / 32768.0f * 16.0f * 9.8f;
            ay = read_int16(4) / 32768.0f * 16.0f * 9.8f;
            az = read_int16(6) / 32768.0f * 16.0f * 9.8f;
        } else if (pkt[1] == 0x52) {
            gx = read_int16(2) / 32768.0f * 2000.0f * static_cast<float>(M_PI) / 180.0f;
            gy = read_int16(4) / 32768.0f * 2000.0f * static_cast<float>(M_PI) / 180.0f;
            gz = read_int16(6) / 32768.0f * 2000.0f * static_cast<float>(M_PI) / 180.0f;
        } else if (pkt[1] == 0x53) {
            roll  = read_int16(2) / 32768.0f * static_cast<float>(M_PI);
            pitch = read_int16(4) / 32768.0f * static_cast<float>(M_PI);
            yaw   = read_int16(6) / 32768.0f * static_cast<float>(M_PI);
            publish_imu();
        }
    }

    void publish_imu() {
        auto msg = sensor_msgs::msg::Imu();

        msg.header.stamp = now();
        msg.header.frame_id = "imu_link";

        msg.linear_acceleration.x = ax;
        msg.linear_acceleration.y = ay;
        msg.linear_acceleration.z = az;

        msg.angular_velocity.x = gx;
        msg.angular_velocity.y = gy;
        msg.angular_velocity.z = gz;

        // Euler (ZYX: roll=X, pitch=Y, yaw=Z) → quaternion
        double cr = std::cos(roll  / 2.0);
        double sr = std::sin(roll  / 2.0);
        double cp = std::cos(pitch / 2.0);
        double sp = std::sin(pitch / 2.0);
        double cy = std::cos(yaw   / 2.0);
        double sy = std::sin(yaw   / 2.0);

        msg.orientation.w = cr * cp * cy + sr * sp * sy;
        msg.orientation.x = sr * cp * cy - cr * sp * sy;
        msg.orientation.y = cr * sp * cy + sr * cp * sy;
        msg.orientation.z = cr * cp * sy - sr * sp * cy;

        // Covariance unknown
        msg.orientation_covariance[0]         = -1.0;
        msg.angular_velocity_covariance[0]    = -1.0;
        msg.linear_acceleration_covariance[0] = -1.0;

        imu_pub_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WT901CNode>());
    rclcpp::shutdown();
}