#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

class SerialBridgeNode : public rclcpp::Node {
public:
    SerialBridgeNode(uint8_t device_id, const std::string &port);

private:
    void update();
    void tx_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);

    int fd_;
    uint8_t device_id_;
    rclcpp::TimerBase::SharedPtr timer_;

    // pub,sub
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr rx_pub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr tx_sub_;
};
