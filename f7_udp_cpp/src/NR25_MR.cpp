/*
RRST NHK2025
汎用機の機構制御

@クリコア
=== Current Parameters ===
0: roller_speed_dribble_ab = 15
1: roller_speed_dribble_cd = 75
2: roller_speed_shoot_ab   = 35
3: roller_speed_shoot_cd   = 35
Shoot State: 0
Dribble State: 0
==========================
*/

// 標準
#include <chrono>
#include <thread>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>

// 自作クラス
#include "include/UDP.hpp"

// 各ローラーの速度を指定(%)
int roller_speed_dribble_ab = 30;
int roller_speed_dribble_cd = 30;
int roller_speed_shoot_ab = 50;
int roller_speed_shoot_cd = 50;
int roller_speed_reload = 15;

// IPアドレスとポートの指定
std::string udp_ip = "192.168.8.216"; // 送信先IPアドレス、宛先マイコンのIPv4アドレスを指定
int udp_port = 5000;                  // 送信元ポート番号、宛先マイコンのポート番号を指定

std::vector<int> data = {0, 0, 0, 0, 0, 0, -1, -1, -1}; // 7~9番を電磁弁制御に転用中（-1 or 1）

// 各機構のシーケンスを格納するクラス
class Action {
public:
    // 事故防止のため、射出機構の展開状況を保存
    static bool ready_for_shoot;
    // 射出機構展開シーケンス
    static void ready_for_shoot_action(UDP &udp) {
        std::cout << "<射出シーケンス開始>" << std::endl;
        std::cout << "展開中..." << std::endl;
        data[6] = 1;
        data[8] = 1;
        data[1] = roller_speed_reload;
        data[2] = roller_speed_reload;
        data[3] = -roller_speed_reload;
        data[4] = -roller_speed_reload;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        data[1] = -roller_speed_shoot_ab;
        data[2] = -roller_speed_shoot_ab;
        data[3] = roller_speed_shoot_cd;
        data[4] = roller_speed_shoot_cd;
        udp.send(data);
        ready_for_shoot = true;
        std::cout << "完了." << std::endl;
    }

    // 射出シーケンス
    static void shoot_action(UDP &udp) {
        std::cout << "シュート" << std::endl;
        data[7] = 1;
        udp.send(data);
        ready_for_shoot = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "格納準備中..." << std::endl;
        data[7] = -1;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "格納中..." << std::endl;
        data[6] = -1;
        data[8] = -1;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        udp.send(data);
        std::cout << "完了." << std::endl;
        std::cout << "<射出シーケンス終了>" << std::endl;
    }

    // ドリブルシーケンス
    static void dribble_action(UDP &udp) {
        std::cout << "<ドリブルシーケンス開始>" << std::endl;
        std::cout << "ドリブル準備中" << std::endl;
        data[1] = roller_speed_dribble_ab;
        data[2] = roller_speed_dribble_ab;
        data[3] = -roller_speed_dribble_cd;
        data[4] = -roller_speed_dribble_cd;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "ドリブル" << std::endl;
        data[8] = 1;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        data[8] = -1;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        udp.send(data);
        std::cout << "完了." << std::endl;
        std::cout << "<ドリブルシーケンス終了>" << std::endl;
    }
};

bool Action::ready_for_shoot = false;

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
        //  float LS_X = -1 * msg->axes[0];
        //  float LS_Y = msg->axes[1];
        //  float RS_X = -1 * msg->axes[3];
        //  float RS_Y = msg->axes[4];

        // bool CROSS = msg->buttons[0];
        bool CIRCLE = msg->buttons[1];
        bool TRIANGLE = msg->buttons[2];
        // bool SQUARE = msg->buttons[3];

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

        // ソフト緊停、配列を初期化し送信、10回試行後にノードを落とす
        if (PS) {
            std::fill(data.begin(), data.end(), 0);                          // 配列をゼロで埋める
            data[6] = data[7] = data[8] = -1;                                // 最後の3つを-1に（電磁弁に転用中の要素のみ-１を代入）
            for (int attempt = 0; attempt < 10; attempt++) {                 // 10回試行
                udp_.send(data);                                             // データ送信
                std::cout << "緊急停止！ 試行" << attempt + 1 << std::endl;  // 試行回数を表示
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 遅延
            }
            rclcpp::shutdown();
        }
        // すべてモーターの場合はこっちを使え
        //  if (PS) {
        //      std::fill(data.begin(), data.end(), 0);                              // 配列をゼロで埋める
        //      for (int attempt = 0; attempt < 10; attempt++) {                     // 10回試行
        //          udp_.send(data);                                                 // データ送信
        //          std::cout << "緊急停止！ 試行" << attempt + 1 << std::endl; // 試行回数を表示
        //          std::this_thread::sleep_for(std::chrono::milliseconds(100));     // 100msの遅延
        //      }
        //      rclcpp::shutdown();
        //  }

        if (CIRCLE) {
            Action::ready_for_shoot_action(udp_);
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        }

        if (Action::ready_for_shoot) {
            Action::shoot_action(udp_);
        }

        // 射出機構が展開済みの場合のみシュートを行う
        if (TRIANGLE && !Action::ready_for_shoot) {
            Action::dribble_action(udp_);
        }

        udp_.send(data);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    UDP udp_;
};

// パラメーター（各ローラーの速度）を受信するクラス
class Params_Listener : public rclcpp::Node {
public:
    Params_Listener()
        : Node("nhk25_pr_listener") {
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "parameter_array", 10,
            std::bind(&Params_Listener::params_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "NHK2025 Parameter Listener");
    }

private:
    void params_listener_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        // 受け取ったパラメーターを各変数に格納
        roller_speed_dribble_ab = msg->data[0];
        roller_speed_dribble_cd = msg->data[1];
        roller_speed_shoot_ab = msg->data[2];
        roller_speed_shoot_cd = msg->data[3];
        // std::cout << roller_speed_dribble_ab;
        // std::cout << roller_speed_dribble_cd;
        // std::cout << roller_speed_shoot_ab;
        // std::cout << roller_speed_shoot_cd << std::endl;
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Executorを指定（MultiThreadedExecutorに変更するかも）
    rclcpp::executors::SingleThreadedExecutor exec;
    auto ps4_listener = std::make_shared<PS4_Listener>(udp_ip, udp_port);
    auto params_listener = std::make_shared<Params_Listener>();
    exec.add_node(ps4_listener);
    exec.add_node(params_listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
