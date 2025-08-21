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

std::vector<int16_t> data(1 + MD + SERVO + SV, 0); // マイコンに送信される配列"data"

class ActuatorSub : public rclcpp::Node {
public:
    ActuatorSub() : Node("actuator_subscriber") {
        sub_ = this->create_subscription<actuator_msg::msg::ActuatorMsg>(
            "actuator_cmd", 10,
            std::bind(&ActuatorSub::callback, this, std::placeholders::_1));
    }

private:
    int get_index(int type, int id) {
        if (type == 1 && id >= 0 && id < MD) {
            return 1 + id; // モータ
        } else if (type == 2 && id >= 0 && id < SERVO) {
            return 1 + MD + id; // サーボ
        } else if (type == 3 && id >= 0 && id < SV) {
            return 1 + MD + SERVO + id; // ソレノイド
        }
        return -1; // 無効
    }

    void callback(const actuator_msg::msg::ActuatorMsg::SharedPtr msg) {
        int idx = get_index(msg->actuator_type, msg->actuator_id);
        if (idx == -1)
            return;

        if (msg->actuator_type == 1) {
            data[idx] = static_cast<int16_t>(msg->motor_duty);
        } else if (msg->actuator_type == 2) {
            data[idx] = static_cast<int16_t>(msg->servo_angle_degree);
        } else if (msg->actuator_type == 3) {
            data[idx] = static_cast<int16_t>(msg->solenoid_state ? 1 : 0);
        }
    }

    rclcpp::Subscription<actuator_msg::msg::ActuatorMsg>::SharedPtr sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorSub>());
    rclcpp::shutdown();
    return 0;
}
