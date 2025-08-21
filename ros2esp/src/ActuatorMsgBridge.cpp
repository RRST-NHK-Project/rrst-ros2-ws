/*
2025, RRST-NHK-Project
各ノードとマイコンの橋渡しを行う。
*/

#include "actuator_msg/msg/actuator_msg.hpp" //自作メッセージ
/*メッセージの定義
# 共通
int32 actuator_id          # アクチュエータの種別ごと（例：MD1とSERVO1は共存可能）
int32 actuator_type        # アクチュエータ種別 (0:デバッグ, 1:モタドラ, 2:サーボ, 3:ソレノイド)
string actuator_type_name  # "MD", "SERVO", "SV"
bool enable                # 有効・無効

# --- モタドラ用 ---
int32 motor_duty           # [%]
int32 motor_target_rpm     # [RPM]
int32 motor_target_pos     #
int32 motor_target_torque  #

# --- サーボ用 ---
int32 servo_angle_degree   # [deg]
int32 servo_speed          #

# --- ソレノイドバルブ用 ---
bool solenoid_state        # [True/False]
*/

#include "rclcpp/rclcpp.hpp"

// 各アクチュエータの総数を定義
#define MD 8
#define SERVO 8
#define SV 8

#define MC_PRINTF 0 // マイコン側のprintfを無効化・有効化(0 or 1)

class ActuatorSub : public rclcpp::Node {
public:
    ActuatorSub() : Node("actuator_subscriber") {
        sub_ = this->create_subscription<actuator_msg::msg::ActuatorMsg>(
            "actuator_cmd", 10,
            std::bind(&ActuatorSub::callback, this, std::placeholders::_1));
    }

private:
    void callback(const actuator_msg::msg::ActuatorMsg::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Recv ID=%d Type=%s",
        //             msg->actuator_id, msg->actuator_type_name.c_str());

        // if (msg->actuator_type == 1) { // モータードライバ
        //     RCLCPP_INFO(this->get_logger(), "Motor Duty=%d, RPM=%d",
        //                 msg->motor_duty, msg->motor_target_rpm);
        // } else if (msg->actuator_type == 2) { // サーボ
        //     RCLCPP_INFO(this->get_logger(), "Servo Angle=%d deg",
        //                 msg->servo_angle_degree);
        // } else if (msg->actuator_type == 3) { // ソレノイド
        //     RCLCPP_INFO(this->get_logger(), "Solenoid State=%s",
        //                 msg->solenoid_state ? "ON" : "OFF");
        // }
    }

    rclcpp::Subscription<actuator_msg::msg::ActuatorMsg>::SharedPtr sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorSub>());
    rclcpp::shutdown();
    return 0;
}
