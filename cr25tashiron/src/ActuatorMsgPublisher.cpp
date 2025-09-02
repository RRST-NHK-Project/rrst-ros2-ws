/*メッセージの定義
# 共通
int32 actuator_id          # アクチュエータの種別ごと（例：MD1とSERVO1は共存可能）
int32 actuator_type        # アクチュエータ種別 (0:デバッグ, 1:モタドラ, 2:サーボ, 3:ソレノイド, 4:ロボマス)
string actuator_type_name  # "MD", "SERVO", "SV", "ROBOMAS"
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

# --- ロボマス用 ---
int32 robomas_target_angle # [degree]
*/
#include "actuator_msg/msg/actuator_msg.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("actuator_publisher");
    auto pub = node->create_publisher<actuator_msg::msg::ActuatorMsg>("actuator_cmd", 10);

    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        actuator_msg::msg::ActuatorMsg msg;

        // // 例: モータードライバにDuty指令
        // msg.actuator_id = 1;
        // msg.actuator_type = 1; // モタドラ
        // msg.actuator_type_name = "MD";
        // msg.enable = true;

        // msg.motor_duty = 50;
        // msg.motor_target_rpm = 1000;

        // 例？: ロボマスに角度指令
        msg.actuator_type = 4;
        msg.actuator_id = 1;
        msg.robomas_target_angle = 90;
        msg.enable = true;

        pub->publish(msg);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
