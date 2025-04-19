/*
RRST-NHK-Project 2025
汎用機の機構制御

二次ビ
int roller_speed_dribble_ab = 10;
int roller_speed_dribble_cd = 62;
int roller_speed_shoot_ab = 35;
int roller_speed_shoot_cd = 35;
int reload = 15;
*/

// 標準
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>

// 自作クラス
#include "include/IP.hpp"
#include "include/UDP.hpp"

#define MC_PRINTF 0 // マイコン側のprintfを無効化・有効化(0 or 1)

// 各ローラーの速度を指定(%)
int roller_speed_dribble_ab = 11;
int roller_speed_dribble_cd = 62;
//ロングシュート4242
//3ポイントシュート3535
int roller_speed_shoot_ab = 35;
int roller_speed_shoot_cd = 35;
int roller_speed_pass_ab = 34;
int roller_speed_pass_cd = 30;
int reload = 15;

std::vector<int16_t> data(19, 0); // マイコンに送信される配列"data"
/*
マイコンに送信される配列"data"
debug: マイコンのprintfを有効化, MD: モータードライバー, TR: トランジスタ
| data[n] | 詳細 | 範囲 |
| ---- | ---- | ---- |
| data[0] | debug | 0 or 1 |
| data[1] | MD1 | -100 ~ 100 |
| data[2] | MD2 | -100 ~ 100 |
| data[3] | MD3 | -100 ~ 100 |
| data[4] | MD4 | -100 ~ 100 |
| data[5] | MD5 | -100 ~ 100 |
| data[6] | MD6 | -100 ~ 100 |
| data[7] | Servo1 | 0 ~ 270 |
| data[8] | Servo2 | 0 ~ 270 |
| data[9] | Servo3 | 0 ~ 270 |
| data[10] | Servo4 | 0 ~ 270 |
| data[11] | TR1 | 0 or 1|
| data[12] | TR2 | 0 or 1|
| data[13] | TR3 | 0 or 1|
| data[14] | TR4 | 0 or 1|
| data[15] | TR5 | 0 or 1|
| data[16] | TR6 | 0 or 1|
| data[17] | TR7 | 0 or 1|
| data[18] | TR8 | 0 or 1|
*/

// 各機構のシーケンスを格納するクラス
class Action {
public:
    // 事故防止のため、射出機構の展開状況を保存
    static bool reload_state;
    static bool shoot_state;
    // 射出機構展開シーケンス
    static void ready_for_shoot_action(UDP &udp) {
        reload_state = true;
        std::cout << "<射出シーケンス開始>" << std::endl;
        std::cout << "展開中..." << std::endl;
        data[11] = 1;
        data[13] = 1;
        data[1] = -reload;
        data[2] = reload;
        data[3] = -reload;
        data[4] = reload;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        udp.send(data);
        std::cout << data[1] << data[2] << data[3] << data[4] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        udp.send(data);
        shoot_state = true;
        std::cout << "射出待機中" << std::endl;
    }

    // 射出シーケンス
    static void shoot_action(UDP &udp) {
        data[1] = roller_speed_shoot_ab;
        data[2] = -roller_speed_shoot_ab;
        data[3] = roller_speed_shoot_cd;
        data[4] = -roller_speed_shoot_cd;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        udp.send(data);
        shoot_state = true;
        std::cout << "完了." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "シュート" << std::endl;
        data[12] = 1;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "格納準備中..." << std::endl;
        data[12] = 0;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "格納中..." << std::endl;
        data[11] = 0;
        data[13] = 0;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        udp.send(data);
        reload_state = false;
        shoot_state = false;

        std::cout << "完了." << std::endl;
        std::cout << "<射出シーケンス終了>" << std::endl;
    }

 // パスシーケンス
    static void pass_action(UDP &udp) {
        data[1] = roller_speed_pass_ab;
        data[2] = -roller_speed_pass_ab;
        data[3] = roller_speed_pass_cd;
        data[4] = -roller_speed_pass_cd;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        udp.send(data);
        shoot_state = true;
        std::cout << "完了." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "パス" << std::endl;
        data[12] = 1;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "格納準備中..." << std::endl;
        data[12] = 0;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "格納中..." << std::endl;
        data[11] = 0;
        data[13] = 0;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        udp.send(data);
        reload_state = false;
        shoot_state = false;

        std::cout << "完了." << std::endl;
        std::cout << "<パスシーケンス終了>" << std::endl;
    }

    // ドリブルシーケンス
    static void dribble_action(UDP &udp) {
        std::cout << "<ドリブルシーケンス開始>" << std::endl;
        std::cout << "ドリブル準備中" << std::endl;
        data[1] = -roller_speed_dribble_ab;
        data[2] = roller_speed_dribble_ab;
        data[3] = -roller_speed_dribble_cd;
        data[4] = roller_speed_dribble_cd;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1200));
        std::cout << "ドリブル" << std::endl;
        data[13] = 1;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        data[13] = 0;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        udp.send(data);
        std::cout << "完了." << std::endl;
        std::cout << "<ドリブルシーケンス終了>" << std::endl;
    }

    // テスト用！！実機で実行するな！！！！
    static void tester(UDP &udp) {
        int tester_time = 150;
        while (1) {
            for (int i = 11; i <= 18; ++i) {
                data[i] = 1;
                udp.send(data);
                std::this_thread::sleep_for(std::chrono::milliseconds(tester_time));
            }
            for (int i = 11; i <= 18; ++i) {
                data[i] = 0;
                udp.send(data);
                std::this_thread::sleep_for(std::chrono::milliseconds(tester_time));
            }
        }
    }
};

bool Action::reload_state = false;
bool Action::shoot_state = false;

class PS4_Listener : public rclcpp::Node {
public:
    PS4_Listener(const std::string &ip, int port)
        : Node("nhk25_mr"), udp_(ip, port) {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy0", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "NHK2025 MR initialized with IP: %s, Port: %d", ip.c_str(),
                    port);
    }

private:
    // コントローラーの入力を取得、使わない入力はコメントアウト推奨
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        //  float LS_X = -1 * msg->axes[0];
        //  float LS_Y = msg->axes[1];
        //  float RS_X = -1 * msg->axes[3];
        //  float RS_Y = msg->axes[4];

        bool CROSS = msg->buttons[0];
        bool CIRCLE = msg->buttons[1];
        bool TRIANGLE = msg->buttons[2];
        bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        // bool UP = msg->axes[7] == 1.0;
        // bool DOWN = msg->axes[7] == -1.0;

        // bool L1 = msg->buttons[4];
        // bool R1 = msg->buttons[5];

        // float L2 = (-1 * msg->axes[2] + 1) / 2;
        // float R2 = (-1 * msg->axes[5] + 1) / 2;

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        data[0] = MC_PRINTF; // マイコン側のprintfを無効化・有効化(0 or 1)

        if (PS) {
            std::fill(data.begin(), data.end(), 0);          // 配列をゼロで埋める
            for (int attempt = 0; attempt < 10; attempt++) { // 10回試行
                udp_.send(data);                             // データ送信
                std::cout << "緊急停止！ 試行" << attempt + 1
                          << std::endl; // 試行回数を表示
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(100)); // 100msの遅延
            }
            rclcpp::shutdown();
        }

        if (CIRCLE && !Action::reload_state) {
            Action::ready_for_shoot_action(udp_);
        }

        // 射出機構が展開済みの場合のみシュートを行う
        if (CROSS && Action::shoot_state) {
            Action::shoot_action(udp_);
        }

        if (TRIANGLE && !Action::shoot_state) {
            Action::dribble_action(udp_);
        }
        
        if (SQUARE && Action::shoot_state) {
            Action::pass_action(udp_);
        }

        // if (OPTION) {
        //     Action::tester(udp_);
        // }

        udp_.send(data);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    UDP udp_;
};

class Params_Listener : public rclcpp::Node {
public:
    Params_Listener() : Node("mr_pr_listener") {
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "parameter_array", 10,
            std::bind(&Params_Listener::params_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),

                    "NHK2025 Parameter Listener initialized");
    }

private:
    void params_listener_callback(
        const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        roller_speed_dribble_ab = msg->data[0];
        roller_speed_dribble_cd = msg->data[1];
        roller_speed_shoot_ab = msg->data[2];
        roller_speed_shoot_cd = msg->data[3];
        std::cout << roller_speed_dribble_ab;
        std::cout << roller_speed_dribble_cd;
        std::cout << roller_speed_shoot_ab;
        std::cout << roller_speed_shoot_cd << std::endl;
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet RRST MR";
    int result = std::system(figletout.c_str());
    if (result != 0) {
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
        std::cerr << "Please install 'figlet' with the following command:"
                  << std::endl;
        std::cerr << "sudo apt install figlet" << std::endl;
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
    }

    rclcpp::executors::SingleThreadedExecutor exec;
    auto ps4_listener = std::make_shared<PS4_Listener>(IP_MR, PORT_MR);
    auto params_listener = std::make_shared<Params_Listener>();
    exec.add_node(ps4_listener);
    exec.add_node(params_listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}