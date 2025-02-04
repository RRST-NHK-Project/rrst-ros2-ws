/*
RRST NHK2025
汎用機の機構制御
*/

#include "include/UDP.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <cmath>
#include <thread>

// 各ローラーの速度を指定(%)
int roller_speed_dribble_ab = 30;
int roller_speed_dribble_cd = 30;
int roller_speed_shoot_ab = 50;
int roller_speed_shoot_cd = 50;
int roller_speed_reload = 10;

int deg;

// IPアドレスとポートの指定
std::string udp_ip = "192.168.8.215"; // 送信先IPアドレス、宛先マイコンで設定したIPv4アドレスを指定
int udp_port = 5000;                  // 送信元ポート番号、宛先マイコンで設定したポート番号を指定

std::vector<int> data = {0, 0, 0, 0, 0, 0, 0, 0, 0}; 

int wheelspeed = 60;
float deadzone = 0.3;

class PS4_Listener : public rclcpp::Node {
public:
    PS4_Listener(const std::string &ip, int port)
        : Node("nhk25_mr_sd"), udp_(ip, port) {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "NHK2025 MR SD initialized with IP: %s, Port: %d", ip.c_str(),
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
        // bool CIRCLE = msg->buttons[1];
        // bool TRIANGLE = msg->buttons[2];
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
        bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        if (PS) {
            std::fill(data.begin(), data.end(), 0);                          // 配列をゼロで埋める
            data[6] = data[7] = data[8] = -1;                                // 最後の3つを-1に
            for (int attempt = 0; attempt < 10; attempt++) {                 // 10回試行
                udp_.send(data);                                             // データ送信
                std::cout << "緊急停止！ 試行" << attempt + 1 << std::endl;  // 試行回数を表示
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 100msの遅延
            }
            rclcpp::shutdown();
        }

        float rad = atan2(LS_Y, LS_X);
        deg = rad * 180 / M_PI;

        if ((fabs(LS_X) <= deadzone) && (fabs(LS_Y) <= deadzone)) {
            data[1] = 0;
            data[2] = 0;
            data[3] = 0;
            data[4] = 0;
        }

        if ((-180 <= deg) && (deg <= -135)) {
            deg = -deg - 135;
        } else {
            deg = 225 - deg;
        }
        std::cout << deg << std::endl;

        data[7] = deg;
        data[1] = wheelspeed * R2;
        data[2] = wheelspeed * R2;
        data[3] = wheelspeed * R2;
        data[4] = wheelspeed * R2;

        if ((fabs(LS_X) <= deadzone) && (fabs(LS_Y) <= deadzone)) {
            data[7] = 135;
        }

        if (LEFT) {
            data[7] = 45;
            data[1] = wheelspeed * R2;
            data[2] = wheelspeed * R2;
            data[3] = wheelspeed * R2;
            data[4] = wheelspeed * R2;
        }
        if (RIGHT) {
            data[7] = 225;
            data[1] = wheelspeed * R2;
            data[2] = wheelspeed * R2;
            data[3] = wheelspeed * R2;
            data[4] = wheelspeed * R2;
        }
        if (UP) {
            data[7] = 135;
            data[1] = wheelspeed * R2;
            data[2] = wheelspeed * R2;
            data[3] = wheelspeed * R2;
            data[4] = wheelspeed * R2;
        }
        if (DOWN) {
            data[7] = 135;
            data[1] = -wheelspeed * R2;
            data[2] = -wheelspeed * R2;
            data[3] = -wheelspeed * R2;
            data[4] = -wheelspeed * R2;
        }

        udp_.send(data);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    UDP udp_;
};

class Servo_Deg_Publisher : public rclcpp::Node
{
public:
  Servo_Deg_Publisher()
  : Node("servo_deg_publisher")
  {
    // Publisherの作成
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("mr_servo_deg", 10);
    
    // タイマーを使って定期的にメッセージをpublish
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&Servo_Deg_Publisher::publish_message, this)
    );
  }

private:
  void publish_message()
  {
    auto message = std_msgs::msg::Int32();
    message.data = deg; 

    //RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
    publisher_->publish(message);  // メッセージをpublish
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto ps4_listener = std::make_shared<PS4_Listener>(udp_ip, udp_port);
    auto servo_deg_publisher = std::make_shared<Servo_Deg_Publisher>();
    exec.add_node(ps4_listener);
    exec.add_node(servo_deg_publisher);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
