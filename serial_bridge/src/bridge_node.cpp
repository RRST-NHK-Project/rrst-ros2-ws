#include "serial_bridge/bridge_node.hpp"
#include <deque>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

constexpr size_t TX16NUM = 8;

SerialBridgeNode::SerialBridgeNode(uint8_t device_id, const std::string &port)
    : Node("serial_bridge_" + std::to_string(device_id)),
      device_id_(device_id) {
    RCLCPP_INFO(this->get_logger(),
                "Device ID 0x%02X → Port %s", device_id, port.c_str());

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
    tty.c_cc[VMIN] = 0;  // read が即返る
    tty.c_cc[VTIME] = 1; // 0.1秒
    tcsetattr(fd_, TCSANOW, &tty);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&SerialBridgeNode::update, this));
}

void SerialBridgeNode::update() {
    constexpr uint8_t START_BYTE = 0xAA;
    static std::deque<uint8_t> rx_buffer;

    uint8_t buf[128];
    int n = read(fd_, buf, sizeof(buf));
    if (n <= 0)
        return;

    for (int i = 0; i < n; i++) {
        rx_buffer.push_back(buf[i]);
    }

    while (rx_buffer.size() >= 4) { // 最小フレームサイズ: START+ID+LEN+CHK
        // START_BYTE 同期
        if (rx_buffer.front() != START_BYTE) {
            rx_buffer.pop_front();
            continue;
        }

        if (rx_buffer.size() < 3)
            return;
        uint8_t length = rx_buffer[2];
        size_t frame_size = 1 + 1 + 1 + length + 1;

        if (rx_buffer.size() < frame_size)
            return; // フレーム未完

        // CHECKSUM計算
        uint8_t checksum = 0;
        for (size_t i = 1; i < 3 + length; i++) {
            checksum ^= rx_buffer[i];
        }

        if (checksum != rx_buffer[3 + length]) {
            RCLCPP_WARN(this->get_logger(),
                        "Checksum mismatch: rx=0x%02X expected=0x%02X",
                        rx_buffer[3 + length], checksum);
            rx_buffer.pop_front(); // 先頭ずらして再試行
            continue;
        }

        // IDチェック
        uint8_t rx_id = rx_buffer[1];
        if (rx_id != device_id_) {
            RCLCPP_WARN(this->get_logger(),
                        "ID mismatch: rx=0x%02X expected=0x%02X",
                        rx_id, device_id_);
            for (size_t i = 0; i < frame_size; i++)
                rx_buffer.pop_front();
            continue;
        }

        // 16bit データ復元
        int16_t values[TX16NUM] = {0};
        for (size_t i = 0; i < TX16NUM && i * 2 + 4 <= frame_size - 1; i++) {
            values[i] = (int16_t)((rx_buffer[3 + i * 2] << 8) |
                                  rx_buffer[3 + i * 2 + 1]);
        }

        // デバッグ表示
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < TX16NUM; i++) {
            oss << values[i];
            if (i + 1 < TX16NUM)
                oss << ", ";
        }
        oss << "]";
        RCLCPP_INFO(this->get_logger(),
                    "[ID 0x%02X] RX DATA: %s",
                    device_id_, oss.str().c_str());

        // 使用済みバイト削除
        for (size_t i = 0; i < frame_size; i++)
            rx_buffer.pop_front();
    }
}
