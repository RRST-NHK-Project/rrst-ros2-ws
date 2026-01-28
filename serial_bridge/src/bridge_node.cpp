#include "serial_bridge/bridge_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include <deque>
#include <fcntl.h>
#include <sstream>
#include <termios.h>
#include <unistd.h>

constexpr size_t TX16NUM = 24;
constexpr size_t RX16NUM = 17;

// コメントそのうち整備します

/*
フレーム構造
[0] START   = 0xAA
[1] ID
[2] LEN     = データ長(byte)
[3..] DATA  = int16 big-endian
[last] CHECKSUM = ID ^ LEN ^ DATA...
*/

SerialBridgeNode::SerialBridgeNode(uint8_t device_id, const std::string &port)
    : Node("serial_bridge_" + std::to_string(device_id)),
      device_id_(device_id),
      fd_(-1) {

    RCLCPP_INFO(this->get_logger(),
                "Device ID 0x%02X → Port %s",
                device_id_, port.c_str());

    // ---------- serial open ----------
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
        return;
    }

    termios tty{};
    tcgetattr(fd_, &tty);
    cfmakeraw(&tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cc[VMIN] = 0;  // non-blocking
    tty.c_cc[VTIME] = 1; // 0.1s
    tcsetattr(fd_, TCSANOW, &tty);

    // ---------- ROS Pub/Sub ----------
    rx_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
        "serial_rx_" + std::to_string(device_id_), 10);

    tx_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "serial_tx_" + std::to_string(device_id_), 10,
        std::bind(&SerialBridgeNode::tx_callback,
                  this, std::placeholders::_1));

    // ---------- timer ----------
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), //短くしすぎるとマイコンの処理が追いつかなくなるので注意
        std::bind(&SerialBridgeNode::update, this));
}

/* ================= RX ================= */

void SerialBridgeNode::update() {
    constexpr uint8_t START_BYTE = 0xAA;
    static std::deque<uint8_t> rx_buffer;

    uint8_t buf[128];
    int n = read(fd_, buf, sizeof(buf));
    if (n <= 0)
        return;

    for (int i = 0; i < n; i++)
        rx_buffer.push_back(buf[i]);

    while (rx_buffer.size() >= 4) {
        // START 同期
        if (rx_buffer.front() != START_BYTE) {
            rx_buffer.pop_front();
            continue;
        }

        uint8_t length = rx_buffer[2];
        size_t frame_size = 1 + 1 + 1 + length + 1;

        if (rx_buffer.size() < frame_size)
            return; // フレーム未完

        // checksum
        uint8_t checksum = 0;
        for (size_t i = 1; i < 3 + length; i++)
            checksum ^= rx_buffer[i];

        if (checksum != rx_buffer[3 + length]) {
            RCLCPP_WARN(this->get_logger(),
                        "Checksum mismatch");
            rx_buffer.pop_front();
            continue;
        }

        // ID check
        uint8_t rx_id = rx_buffer[1];
        if (rx_id != device_id_) {
            RCLCPP_WARN(this->get_logger(),
                        "ID mismatch rx=0x%02X expected=0x%02X",
                        rx_id, device_id_);
            for (size_t i = 0; i < frame_size; i++)
                rx_buffer.pop_front();
            continue;
        }

        // 16bit data decode
        int16_t values[RX16NUM] = {0};
        size_t data16_count = std::min((size_t)(length / 2), TX16NUM);

        for (size_t i = 0; i < data16_count; i++) {
            values[i] = (int16_t)((rx_buffer[3 + i * 2] << 8) |
                                  rx_buffer[3 + i * 2 + 1]);
        }

        // publish
        std_msgs::msg::Int16MultiArray msg;
        msg.data.assign(values, values + RX16NUM);
        rx_pub_->publish(msg);

        // debug log
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < RX16NUM; i++) {
            oss << values[i];
            if (i + 1 < RX16NUM)
                oss << ", ";
        }
        oss << "]";

        RCLCPP_INFO(this->get_logger(),
                    "[ID 0x%02X] RX DATA: %s",
                    device_id_, oss.str().c_str());

        // consume
        for (size_t i = 0; i < frame_size; i++)
            rx_buffer.pop_front();
    }
}

/* ================= TX ================= */

void SerialBridgeNode::tx_callback(
    const std_msgs::msg::Int16MultiArray::SharedPtr msg) {

    if (msg->data.size() < TX16NUM)
        return;

    constexpr uint8_t START_BYTE = 0xAA;
    constexpr uint8_t LEN = TX16NUM * 2;

    uint8_t frame[1 + 1 + 1 + LEN + 1];

    frame[0] = START_BYTE;
    frame[1] = device_id_;
    frame[2] = LEN;

    for (size_t i = 0; i < TX16NUM; i++) {
        frame[3 + i * 2] = (msg->data[i] >> 8) & 0xFF;
        frame[3 + i * 2 + 1] = msg->data[i] & 0xFF;
    }

    uint8_t checksum = 0;
    for (size_t i = 1; i < 3 + LEN; i++)
        checksum ^= frame[i];

    frame[3 + LEN] = checksum;

    write(fd_, frame, sizeof(frame));
}
