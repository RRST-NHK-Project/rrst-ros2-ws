#include "rclcpp/rclcpp.hpp"
#include "actuator_msg/msg/actuator_msg.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("actuator_publisher");
    auto pub = node->create_publisher<actuator_msg::msg::ActuatorMsg>("actuator_cmd", 10);

    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        actuator_msg::msg::ActuatorMsg msg;

        // 例: モータードライバにDuty指令
        msg.actuator_id = 1;
        msg.actuator_type = 1; // モタドラ
        msg.actuator_type_name = "MD";
        msg.enable = true;

        msg.motor_duty = 50;
        msg.motor_target_rpm = 1000;

        pub->publish(msg);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
