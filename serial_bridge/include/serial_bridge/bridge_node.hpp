#pragma once
#include <rclcpp/rclcpp.hpp>
#include <string>

class SerialBridgeNode : public rclcpp::Node {
public:
    SerialBridgeNode(uint8_t device_id, const std::string &port);

private:
    void update();
    int fd_ = -1;
    rclcpp::TimerBase::SharedPtr timer_;
    uint8_t device_id_;
};
