/*
2025, RRST-NHK-Project
各ノードとマイコンの橋渡しを行う。
*/

#include "actuator_msg/msg/actuator_msg.hpp" //自作メッセージ
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



#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
// **複数のESPを使用する場合はIDを変更** //
#define ID 0
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//

// 各アクチュエータの総数を定義
#define MD 8
#define SERVO 8
#define SV 8
#define ROBOMAS 8

#define MC_PRINTF 1 // マイコン側のprintfを無効化・有効化(0 or 1)

std::vector<int16_t> data(1 + MD + SERVO + SV + ROBOMAS, 0); // マイコンに送信される配列"data"

// ノード名とトピック名の定義（ID付き）
const std::string node_name = "esp32node_" + std::to_string(ID);
const std::string subscriber_topic_name = "from_esp32_" + std::to_string(ID);
const std::string publisher_topic_name = "to_esp32_" + std::to_string(ID);

class ActuatorSub : public rclcpp::Node {
public:
    ActuatorSub() : Node(node_name.c_str()) {
        sub_ = this->create_subscription<actuator_msg::msg::ActuatorMsg>(
            "actuator_cmd", 10,
            std::bind(&ActuatorSub::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(publisher_topic_name.c_str(), 10);
    }

private:
    int get_index(int type, int id) {
        if (type == 1 && id >= 0 && id < MD) {
            return 1 + id; // モータ
        } else if (type == 2 && id >= 0 && id < SERVO) {
            return 1 + MD + id; // サーボ
        } else if (type == 3 && id >= 0 && id < SV) {
            return 1 + MD + SERVO + id; // ソレノイド
        }else if (type == 4 && id >= 0 && id < ROBOMAS) {
            return 1 + MD + SERVO + SV + id; // ロボマス
        }

        return -1; // 無効
    }

    void callback(const actuator_msg::msg::ActuatorMsg::SharedPtr msg) {

        data[0] = MC_PRINTF; // マイコン側のprintfを無効化・有効化(0 or 1)

        int idx = get_index(msg->actuator_type, msg->actuator_id);
        if (idx == -1)
            return;

        if (msg->actuator_type == 1) {
            data[idx] = static_cast<int16_t>(msg->motor_duty);
        } else if (msg->actuator_type == 2) {
            data[idx] = static_cast<int16_t>(msg->servo_angle_degree);
        } else if (msg->actuator_type == 3) {
            data[idx] = static_cast<int16_t>(msg->solenoid_state ? 1 : 0);
        } else if (msg->actuator_type == 4) {
            data[idx] = static_cast<int16_t>(msg->robomas_target_angle);
        }

        // デバッグ用（for文でcoutするとカクつく）
        // std::cout << data[0] << ", " << data[1] << ", " << data[2] << ", " << data[3] << ", ";
        // std::cout << data[4] << ", " << data[5] << ", " << data[6] << ", " << data[7] << ", ";
        // std::cout << data[8] << ", " << data[9] << ", " << data[10] << ", " << data[11] << ", ";
        // std::cout << data[12] << ", " << data[13] << ", " << data[14] << ", " << data[15] << ", ";
        // std::cout << data[16] << ", " << data[17] << ", " << data[18] << ", " << data[19] << ", ";
        // std::cout << data[20] << ", " << data[21] << ", " << data[22] << ", " << data[23] << ", ";
        // std::cout << data[24] << std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));

        auto msg_out = std_msgs::msg::Int32MultiArray();
        msg_out.data.assign(data.begin(), data.end());
        pub_->publish(msg_out);
    }

    rclcpp::Subscription<actuator_msg::msg::ActuatorMsg>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorSub>());
    rclcpp::shutdown();
    return 0;
}
