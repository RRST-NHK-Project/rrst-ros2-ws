#include "rclcpp/rclcpp.hpp"
#include "actuator_msg/msg/actuator_msg.hpp"

class ActuatorSub : public rclcpp::Node
{
public:
    ActuatorSub() : Node("actuator_subscriber")
    {
        sub_ = this->create_subscription<actuator_msg::msg::ActuatorMsg>(
            "actuator_cmd", 10,
            std::bind(&ActuatorSub::callback, this, std::placeholders::_1));
    }

private:
    void callback(const actuator_msg::msg::ActuatorMsg::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Recv ID=%d Type=%s",
                    msg->actuator_id, msg->actuator_type_name.c_str());

        if (msg->actuator_type == 1) { // モータードライバ
            RCLCPP_INFO(this->get_logger(), "Motor Duty=%d, RPM=%d",
                        msg->motor_duty, msg->motor_target_rpm);
        } else if (msg->actuator_type == 2) { // サーボ
            RCLCPP_INFO(this->get_logger(), "Servo Angle=%d deg",
                        msg->servo_angle_degree);
        } else if (msg->actuator_type == 3) { // ソレノイド
            RCLCPP_INFO(this->get_logger(), "Solenoid State=%s",
                        msg->solenoid_state ? "ON" : "OFF");
        } else if (msg->actuator_type == 4) { // ロボマス
            RCLCPP_INFO(this->get_logger(), "Robomas Target Angle=%d",
                        msg->robomas_target_angle);
        }
    }

    rclcpp::Subscription<actuator_msg::msg::ActuatorMsg>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorSub>());
    rclcpp::shutdown();
    return 0;
}
