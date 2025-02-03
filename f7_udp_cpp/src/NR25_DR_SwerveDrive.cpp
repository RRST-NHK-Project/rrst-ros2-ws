/*
RRST NHK2025
汎用機の機構制御
*/

#include <chrono>
#include <thread>
#include <cmath>
#include "include/UDP.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

// 各ローラーの速度を指定(%)
int roller_speed_dribble_ab = 30;
int roller_speed_dribble_cd = 30;
int roller_speed_shoot_ab = 50;
int roller_speed_shoot_cd = 50;
int roller_speed_reload = 10;

// IPアドレスとポートの指定
std::string udp_ip = "192.168.8.215"; // 送信先IPアドレス、宛先マイコンで設定したIPv4アドレスを指定
int udp_port = 5000;                  // 送信元ポート番号、宛先マイコンで設定したポート番号を指定

std::vector<int> data = {0, 0, 0, 0, 0, 0, -1, -1, -1}; // 7~9番を電磁弁制御に転用中（-1 or 1）

int wheelspeed = 20;
float deadzone = 0.3 ;


class PS4_Listener : public rclcpp::Node {
public:
    PS4_Listener(const std::string &ip, int port)
        : Node("nhk25_mr"), udp_(ip, port) {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "NHK2025 MR initialized with IP: %s, Port: %d", ip.c_str(),
                    port);
    }

private:
    // コントローラーの入力を取得、使わない入力はコメントアウト推奨
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
          float LS_X = -1 * msg->axes[0];
          float LS_Y = msg->axes[1];
        //  float RS_X = -1 * msg->axes[3];
        //  float RS_Y = msg->axes[4];


        // bool CROSS = msg->buttons[0];
        //bool CIRCLE = msg->buttons[1];
        //bool TRIANGLE = msg->buttons[2];
        // bool SQUARE = msg->buttons[3];

         bool LEFT = msg->axes[6] == 1.0;
         bool RIGHT = msg->axes[6] == -1.0;
         bool UP = msg->axes[7] == 1.0;
         bool DOWN = msg->axes[7] == -1.0;

        // bool L1 = msg->buttons[4];
        // bool R1 = msg->buttons[5];

        // float L2 = (-1 * msg->axes[2] + 1) / 2;
         float R2 = (-1 * msg->axes[5] + 1) / 2;

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
         //bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];
        /*
                if (PS) {
                    while (1) {
                        std::fill(data.begin(), data.end(), 0);
                        udp_.send(data);
                        std::cout << "！緊急停止中！" << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                }
        */
        if ((fabs(LS_X) <= deadzone)&&(fabs(LS_Y) <= deadzone)){
            LS_X = 0;
            LS_Y = 0;
        }

        if (
            (fabs(LS_X) <= deadzone)
            && (fabs(LS_Y) <= deadzone)
        ){
            data[1] = 0;
            data[2] = 0;
            data[3] = 0;
            data[4] = 0;
        }

        float rad = atan2(LS_Y, LS_X);
        float deg = int(rad * 180 / M_PI);

        if ((-180 <= deg)&&(deg <= -135)){
            deg= - deg -135;
            }
        else{
            deg = 225-deg;
        }
        std::cout << "完了" << deg <<std::endl;

        data[7] = deg;
        data[1] = wheelspeed*R2;
        data[2] = wheelspeed*R2;
        data[3] = wheelspeed*R2;
        data[4] = wheelspeed*R2;

    
        if (LEFT) {
            data[7] = 45;
            data[1] = wheelspeed*R2;
            data[2] = wheelspeed*R2;
            data[3] = wheelspeed*R2;
            data[4] = wheelspeed*R2;
        }
        if (RIGHT) {
            data[7] = 225;
            data[1] = wheelspeed*R2;
            data[2] = wheelspeed*R2;
            data[3] = wheelspeed*R2;
            data[4] = wheelspeed*R2;
        }
        if (UP) {
            data[7] = 135;
            data[1] = wheelspeed*R2;
            data[2] = wheelspeed*R2;
            data[3] = wheelspeed*R2;
            data[4] = wheelspeed*R2;
        }
        if (DOWN) {
            data[7] = 135;
            data[1] = -wheelspeed*R2;
            data[2] = -wheelspeed*R2;
            data[3] = -wheelspeed*R2;
            data[4] = -wheelspeed*R2;
        }

        udp_.send(data);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    UDP udp_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto ps4_listener = std::make_shared<PS4_Listener>(udp_ip, udp_port);
    exec.add_node(ps4_listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
