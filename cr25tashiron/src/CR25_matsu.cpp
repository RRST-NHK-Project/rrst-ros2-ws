/*
RRST-NHK-Project 2025
PS4コントローラーの入力を取得するサンプルプログラム
esp32マイコンにアクチュエータ指令を送るサンプルプログラム
*/

// 標準
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "actuator_msg/msg/actuator_msg.hpp"
#include <vector>
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

#define MC_PRINTF 0 // マイコン側のprintfを無効化・有効化(0 or 1)


std::vector<int32_t> data(33, 0); // マイコンに送信される配列"data"

class PS4_Listener : public rclcpp::Node {
public:
    PS4_Listener()
        : Node("ps4_listener") {

        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this,
                      std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("cr25_matsu", 10);

        RCLCPP_INFO(this->get_logger(),
                    "PS4 Listener initialized");
    }

private:
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        //  float LS_X = -1 * msg->axes[0];
        //  float LS_Y = msg->axes[1];
        //  float RS_X = -1 * msg->axes[3];
        //  float RS_Y = msg->axes[4];

        bool CROSS = msg->buttons[0];
        bool CIRCLE = msg->buttons[1];
        // bool TRIANGLE = msg->buttons[2];
        // bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        // bool UP = msg->axes[7] == 1.0;
        // bool DOWN = msg->axes[7] == -1.0;

        // bool L1 = msg->buttons[4];
        //  bool R1 = msg->buttons[5];

        // float L2 = (-1 * msg->axes[2] + 1) / 2;
        //float R2 = (-1 * msg->axes[5] + 1) / 2;

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        data[0] = MC_PRINTF; // マイコン側のprintfを無効化・有効化(0 or 1)

        if (CIRCLE) {
           // std::cout << "CIRCLE" << std::endl;
           data[24] = 90; //ロボマスモータに90度指令
           // std::cout << data[24] << std::endl;

         publish_data();
         }
         if (CROSS) {
           //std::cout << "CIRCLE" << std::endl;
           data[24] = 0; //ロボマスモータに90度指令
           // std::cout << data[24] << std::endl;

         publish_data();
         }

         std::cout << data[24] << std::endl;
        publish_data();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    void publish_data() {
        auto msg = std_msgs::msg::Int32MultiArray();
        msg.data.reserve(data.size());
        for (auto &v : data) {
            msg.data.push_back(static_cast<int32_t>(v));
        }
        publisher_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
};



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    
    //exec.spin();
    rclcpp::executors::MultiThreadedExecutor exec;
     auto ps4_listener = std::make_shared<PS4_Listener>();
  auto node = rclcpp::Node::make_shared("actuator_publisher");
    auto pub = node->create_publisher<actuator_msg::msg::ActuatorMsg>("actuator_cmd", 10);
    auto timer = node->create_wall_timer(std::chrono::milliseconds(100), [pub](){
    
    //exec.add_node(ps4_listener);
   
        actuator_msg::msg::ActuatorMsg msg;

        // 例: モータードライバにDuty指令
        // msg.actuator_id = 1;
        // msg.actuator_type = 1; // モタドラ
        // msg.actuator_type_name = "MD";
        // msg.enable = true;

        // msg.motor_duty = 50;
        // msg.motor_target_rpm = 1000;

        // 例: robomas指令
        msg.actuator_id = 1;
        msg.actuator_type = 4; // ロボマス
        msg.actuator_type_name = "ROBOMAS";
        msg.enable = true;

        msg.robomas_target_angle = 90; // 目標角度90度
        pub->publish(msg);
    });
   
    exec.add_node(ps4_listener);
    exec.add_node(node);
    exec.spin();
    
    rclcpp::shutdown();
    return 0;
}