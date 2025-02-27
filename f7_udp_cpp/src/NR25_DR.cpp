/*
RRST NHK2025
ダンク機の機構制御
*/

// 標準
#include <chrono>
#include <thread>
#include <unistd.h>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>

// 自作クラス
#include "include/UDP.hpp"

#define MC_PRINTF 1 // マイコン側のprintfを無効化・有効化(0 or 1)

// IPアドレスとポートの指定
std::string udp_ip = "192.168.0.218"; // 送信先IPアドレス、宛先マイコンで設定したIPv4アドレスを指定
int udp_port = 5000;                  // 送信元ポート番号、宛先マイコンで設定したポート番号を指定

std::vector<int16_t> data(19, 0); // 1~5番を電磁弁制御に転用中（-1 or 1）

// 各機構のシーケンスを格納するクラス
class Action {
public:
    // 事故防止のため、ブームの展開状況を保存
    static bool ready_for_dunk;

    static void ready_for_dunk_action(UDP &udp) {
        std::cout << "１段階展開[11]" << std::endl;
        data[11] = 1;
        udp.send(data);
        ready_for_dunk = true;
        std::cout << "完了." << std::endl;
    }

    static void dunk_shoot_action(UDP &udp) {
        std::cout << "<ダンクシーケンス開始>" << std::endl;

        std::cout << "２段階展開[12]＋トリガー[13]" << std::endl;
        data[12] = 1;
        data[13] = 1;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        std::cout << "ストッパ[14]" << std::endl;
        data[14] = 1;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cout << "格納[15]" << std::endl;
        data[15] = 1;
        std::cout << "サーボ[7]" << std::endl;
        data[7] = 0;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 要調整

        std::cout << "１段階格納[11]＋２段階格納[12]" << std::endl;
        data[11] = 0;
        data[12] = 0;
        udp.send(data);
        ready_for_dunk = false;
        std::cout << "完了." << std::endl;
        std::cout << "<ダンクシーケンス終了>" << std::endl;
    }
};
bool Action::ready_for_dunk = false;

class PS4_Listener : public rclcpp::Node {
public:
    PS4_Listener(const std::string &ip, int port)
        : Node("nhk25_dr"), udp_(ip, port) {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy1", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this,
                      std::placeholders::_1));
        // figletでノード名を表示
        std::string figletout = "figlet RRST DR";
        int result = std::system(figletout.c_str());
        if (result != 0) {
            std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            std::cerr << "Please install 'figlet' with the following command:" << std::endl;
            std::cerr << "sudo apt install figlet" << std::endl;
            std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        }
        RCLCPP_INFO(this->get_logger(),
                    "NHK2025 DR initialized with IP: %s, Port: %d", ip.c_str(),
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

        data[0] = MC_PRINTF; // マイコン側のprintfを無効化・有効化(0 or 1)

        if (PS) {
            std::fill(data.begin(), data.end(), 0);                          // 配列をゼロで埋める
            for (int attempt = 0; attempt < 10; attempt++) {                 // 10回試行
                udp_.send(data);                                             // データ送信
                std::cout << "緊急停止！ 試行" << attempt + 1 << std::endl;  // 試行回数を表示
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 100msの遅延
            }
            rclcpp::shutdown();
        }

        if (TRIANGLE) {
            Action::ready_for_dunk_action(udp_);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        if (CIRCLE && Action::ready_for_dunk) {
            Action::dunk_shoot_action(udp_);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        // if (SQUARE) {
        //     data[8] = 45;
        //     udp_.send(data);
        //     //std::cout << "S" << std::endl;
        //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // }

        udp_.send(data);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    UDP udp_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto ps4_listener = std::make_shared<PS4_Listener>(udp_ip, udp_port);
    // auto params_listener = std::make_shared<Params_Listener>();
    exec.add_node(ps4_listener);
    // exec.add_node(params_listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}