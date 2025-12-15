#include "serial_bridge/bridge_node.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

constexpr size_t TX16NUM = 8;
constexpr size_t FRAME_SIZE = 1 + TX16NUM * 2;

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

    // read() が即返る設定
    tty.c_cc[VMIN] = FRAME_SIZE;
    tty.c_cc[VTIME] = 1; // 0.1秒

    tcsetattr(fd_, TCSANOW, &tty);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&SerialBridgeNode::update, this));
}

void SerialBridgeNode::update() {

    uint8_t buf[FRAME_SIZE];

    int n = read(fd_, buf, FRAME_SIZE);
    if (n != FRAME_SIZE) {
        return; // フレーム未満は無視
    }

    // --- IDチェック ---
    uint8_t rx_id = buf[0];
    if (rx_id != device_id_) {
        RCLCPP_WARN(this->get_logger(),
                    "ID mismatch: rx=0x%02X expected=0x%02X",
                    rx_id, device_id_);
        return;
    }

    // --- 16bitデータ復元 ---
    int16_t values[TX16NUM];
    for (size_t i = 0; i < TX16NUM; i++) {
        values[i] =
            (int16_t)((buf[1 + i * 2] << 8) |
                      buf[1 + i * 2 + 1]);
    }

    // --- デバッグ表示 ---
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
}
