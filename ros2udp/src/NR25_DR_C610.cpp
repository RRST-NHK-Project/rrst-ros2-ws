/*
2025/02/15
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

#define MC_PRINTF 0 // マイコン側のprintfを無効化・有効化(0 or 1)

int motor1 = 50;
int motor2 = 50;
int motor3 = 50;
int motor4 = 50;
int hcsr04 = 0;
bool UP_state = false;

// IPアドレスとポートの指定
std::string udp_ip = "192.168.0.218"; // 送信先IPアドレス、宛先マイコンで設定したIPv4アドレスを指定
int udp_port = 5000;                  // 送信元ポート番号、宛先マイコンで設定したポート番号を指定

std::vector<int16_t> data(19, 0);
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
    // 事故防止のため、ブームの展開状況を保存
    static bool ready_for_dunk;

    static void ready_for_dunk_action(UDP &udp) {
        std::cout << "ダンク待機開始" << std::endl;
        std::cout << "１段階展開[1]" << std::endl;
        data[11] = 1;
        udp.send(data);
        ready_for_dunk = true;
        std::cout << "完了." << std::endl;
    }
    // 16は一番上の掴むところ
    static void dunk_shoot_action(UDP &udp) {
        std::cout << "<ダンクシーケンス開始>" << std::endl;

        std::cout << "２段階展開[15]＋トリガー[13]" << std::endl;
        data[15] = 1;

        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cout << "２段階展開[15]＋トリガー[13]" << std::endl;

        data[13] = 1;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "ストッパ[14]" << std::endl;
        data[14] = 1;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(730));
        // シュートはsleep 660
        std::cout << "格納[18]+チルト展開[16]" << std::endl;
        data[18] = 1;
        data[16] = 1;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 要調整
        std::cout << "１段階格納[11]＋２段階格納[15]＋チルト格納[16]" << std::endl;
        data[11] = 0;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 要調整

        data[15] = 0;
        data[16] = 0;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 要調整

        std::cout << "初期状態" << std::endl;
        data[13] = 0;
        udp.send(data);
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 要調整
        ready_for_dunk = false;
        std::cout << "完了." << std::endl;
        std::cout << "<ダンクシーケンス終了>" << std::endl;
    }

    static void pass_shoot_action(UDP &udp) { // パス、シュート
        std::cout << "パス、シュート開始" << std::endl;
        std::cout << "１段階展開[11]＋２段階展開[15]" << std::endl;
        data[11] = 1;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        data[15] = 1;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(400)); // 要調整
        std::cout << "ハンド展開" << std::endl;
        data[16] = 1;
        udp.send(data);
        std::cout << "１段階格納[11]＋２段階格納[15]" << std::endl;
        data[15] = 0;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 要調整
        data[11] = 0;
        udp.send(data);
        std::cout << "パス、シュート完了" << std::endl;
    }

    static void dribble_action(UDP &udp) {
        std::cout << "ドリブル開始" << std::endl;
        std::cout << "１段階展開[11]＋２段階展開[15]" << std::endl;
        data[11] = 1;
        data[15] = 1;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(730)); // 要調整
        std::cout << "<ドリブル[2],[3]>" << std::endl;
        data[16] = 1;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 要調整
        std::cout << "１段階格納[11]＋２段階格納[15]" << std::endl;
        data[11] = 0;
        data[15] = 0;
        udp.send(data);
        std::cout << "ドリブル完了" << std::endl;
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

        bool CROSS = msg->buttons[0];
        bool CIRCLE = msg->buttons[1];
        // bool TRIANGLE = msg->buttons[2];
        bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        // bool DOWN = msg->axes[7] == -1.0;

        // bool L1 = msg->buttons[4];
        // bool R1 = msg->buttons[5];

        float L2 = (-1 * msg->axes[2] + 1) / 2;
        // float R2 = (-1 * msg->axes[5] + 1) / 2;

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];
        static bool last_UP = false; // 前回の状態を保持する static 変数
        // OPTION のラッチ状態を保持する static 変数（初期状態は OFF とする）
        static bool UP_latch = false;

        data[0] = MC_PRINTF; // マイコン側のprintfを無効化・有効化(0 or 1)

        if (PS) {
            std::fill(data.begin(), data.end(), 0);                          // 配列をゼロで埋める                                      // 最後の3つを-1に
            for (int attempt = 0; attempt < 10; attempt++) {                 // 10回試行
                udp_.send(data);                                             // データ送信
                std::cout << "緊急停止！ 試行" << attempt + 1 << std::endl;  // 試行回数を表示
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 100msの遅延
            }
            rclcpp::shutdown();
        }
        if (UP && !last_UP) {
            UP_latch = !UP_latch;
        }
        // if (PS) {
        //     std::fill(data.begin(), data.end(), 0);                              // 配列をゼロで埋める
        //     for (int attempt = 0; attempt < 10; attempt++) {                     // 10回試行
        //         udp_.send(data);                                                 // データ送信
        //         std::cout << "緊急停止！ 試行" << attempt + 1 << std::endl; // 試行回数を表示
        //         std::this_thread::sleep_for(std::chrono::milliseconds(100));     // 100msの遅延
        //     }
        //     rclcpp::shutdown();
        // }

        if (SQUARE) {
            Action::ready_for_dunk_action(udp_);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        if (CIRCLE && Action::ready_for_dunk) {
            Action::dunk_shoot_action(udp_);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        // if (TRIANGLE) {
        //     // std::cout << "<ロボマス回転>" << std::endl;
        //     Action::dribble_action(udp_);
        //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // }
        // if (CROSS) {
        //     Action::pass_shoot_action(udp_);
        //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // }

        // １段階展開[11]＋２段階展開[15] を同時に行うボタン
        if (L2 < 0.5) {
            data[11] = 0;
            data[15] = 0;
            // std::cout << "格納"<< std::endl;
            //  std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        if (L2 >= 0.5) {
            data[11] = 1;
            data[15] = 1;
            // std::cout << "展開"<< std::endl;
            //  std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        if (CROSS) {
            data[7] = 90;
        } else {
            data[7] = 0;
        }

        last_UP = UP;
        UP_state = UP_latch;
        // std::cout << data[16] << std::endl;
        udp_.send(data);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    UDP udp_;
};

class Params_Listener : public rclcpp::Node {
public:
    Params_Listener()
        : Node("nhk25_pr_listener") {
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "dr_parameter_array", 10,
            std::bind(&Params_Listener::params_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "NHK2025 Parameter Listener");
    }

private:
    void params_listener_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        motor1 = msg->data[0];
        motor2 = msg->data[1];
        motor3 = msg->data[2];
        motor4 = msg->data[3];
        std::cout << motor1;
        std::cout << motor2;
        std::cout << motor3;
        std::cout << motor4 << std::endl;
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

class hcsr04_Listener : public rclcpp::Node {
public:
    hcsr04_Listener()
        : Node("nhk25_dr_hcsr04") {
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "hcsr04", 10,
            std::bind(&hcsr04_Listener::hcsr04_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "NHK2025 HCSR04 Listener");
    }

private:
    void hcsr04_listener_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        hcsr04 = msg->data[4]; // FIXME: 適切なインデックスに変更
        std::cout << hcsr04 << std::endl;
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet RRST DR";
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
    auto ps4_listener = std::make_shared<PS4_Listener>(udp_ip, udp_port);
    auto params_listener = std::make_shared<Params_Listener>();
    exec.add_node(ps4_listener);
    exec.add_node(params_listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}