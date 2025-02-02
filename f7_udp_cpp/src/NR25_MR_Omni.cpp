/*
ビルドを通すためだけに記述、書き換えろ
*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "include/UDP.hpp"
#include <chrono>
#include <thread>

int roller_speed_dribble_ab = 30;
int roller_speed_dribble_cd = 30;
int roller_speed_shoot_ab = 50;
int roller_speed_shoot_cd = 50;
int roller_speed_reload = 10;

std::string udp_ip = "192.168.8.216";
int udp_port = 5000;

class Action
{
public:
    static bool ready_for_shoot;

    static void ready_for_shoot_action(UDP &udp)
    {
        std::cout << "<射出シーケンス開始>" << std::endl;
        std::cout << "展開中..." << std::endl;
        data[6] = 1;
        data[8] = 1;
        data[1] = roller_speed_reload;
        data[2] = roller_speed_reload;
        data[3] = -roller_speed_reload;
        data[4] = -roller_speed_reload;
        udp.send();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        udp.send();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        data[1] = -roller_speed_shoot_ab;
        data[2] = -roller_speed_shoot_ab;
        data[3] = roller_speed_shoot_cd;
        data[4] = roller_speed_shoot_cd;
        udp.send();
        ready_for_shoot = true;
        std::cout << "完了." << std::endl;
    }

    static void shoot_action(UDP &udp)
    {
        std::cout << "シュート" << std::endl;
        data[7] = 1;
        udp.send();
        ready_for_shoot = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "格納準備中..." << std::endl;
        data[7] = -1;
        udp.send();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "格納中..." << std::endl;
        data[6] = -1;
        data[8] = -1;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        udp.send();
        std::cout << "完了." << std::endl;
        std::cout << "<射出シーケンス終了>" << std::endl;
    }

    static void dribble_action(UDP &udp)
    {
        std::cout << "<ドリブルシーケンス開始>" << std::endl;
        std::cout << "ドリブル準備中" << std::endl;
        data[1] = roller_speed_dribble_ab;
        data[2] = roller_speed_dribble_ab;
        data[3] = -roller_speed_dribble_cd;
        data[4] = -roller_speed_dribble_cd;
        udp.send();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "ドリブル" << std::endl;
        data[8] = 1;
        udp.send();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        data[8] = -1;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        udp.send();
        std::cout << "完了." << std::endl;
        std::cout << "<ドリブルシーケンス終了>" << std::endl;
    }
};

bool Action::ready_for_shoot = false;

class PS4_Listener : public rclcpp::Node
{
public:
    PS4_Listener(const std::string &ip, int port)
        : Node("nhk25_mr"), udp_(ip, port)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&PS4_Listener::ps4_listener_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "NHK2025 MR initialized with IP: %s, Port: %d", ip.c_str(), port);
    }

private:
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        bool CIRCLE = msg->buttons[1];
        bool TRIANGLE = msg->buttons[2];

        if (CIRCLE)
        {
            Action::ready_for_shoot_action(udp_);
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }

        if (Action::ready_for_shoot)
        {
            Action::shoot_action(udp_);
        }

        if (TRIANGLE && !Action::ready_for_shoot)
        {
            Action::dribble_action(udp_);
        }

        udp_.send();
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    UDP udp_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto ps4_listener = std::make_shared<PS4_Listener>(udp_ip, udp_port);
    exec.add_node(ps4_listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
