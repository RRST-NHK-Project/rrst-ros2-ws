#include "serial_bridge/bridge_node.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

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

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&SerialBridgeNode::update, this));
}

void SerialBridgeNode::update() {
    char buf[128];
    int n = read(fd_, buf, sizeof(buf));
    if (n > 0) {
        std::string msg(buf, n);
        RCLCPP_INFO(this->get_logger(),
                    "[ID 0x%02X] RX: %s", device_id_, msg.c_str());
    }
}
