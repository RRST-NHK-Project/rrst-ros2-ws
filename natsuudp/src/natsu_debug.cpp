/*
RRST-NHK-Project 2010 夏ロボ
機構制御
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

#define MC_PRINTF 1 // マイコン側のprintfを無効化・有効化(0 or 1)

// 各機構の速さの指定(%)
int speed_lift;

// 射出機構の速
int speed_shoot = 0;

int z_speed = 75; // Z軸の速度

bool init_all_state = false; // 機構全体の初期化状態保存

int SHOOTMODE = 0;

bool shoot_enable = false;

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
| data[7] | Servo1 | 0 ~ 80 |
| data[8] | Servo2 | 0 ~ 80 |
| data[9] | Servo3 | 0 ~ 80 |
| data[10] | Servo4 | 0 ~ 80 |
| data[11] | TR1 | 0 or 1|  //VGOAL
| data[12] | TR2 | 0 or 1|
| data[13] | TR3 | 0 or 1|  //ポンプ１
| data[14] | TR4 | 0 or 1|   //ポンプ２
| data[15] | TR5 | 0 or 1|  //シリンダ
| data[16] | TR6 | 0 or 1|
| data[17] | TR7 | 0 or 1|
| data[18] | TR8 | 0 or 1|

*/

// 各機構のシーケンスを格納するクラス
class Action {
public:
    // static bool folk_state; // フォークの状態保存
    static bool folk_init_state; // フォークが初期化されているか保存

    static void all_init(UDP &udp) {
        data[7] = 90;
        udp.send(data);
        std::cout << "機構初期化" << std::endl;
    }

    static void folk_init(UDP &udp) {
        data[1] = 0; // ポンプ
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        data[7] = 90; // サーボ
        data[8] = 0;  // サーボ
        udp.send(data);
        std::cout << "フォーク初期化" << std::endl;
        folk_init_state = true;
    }

    // static void folk_tenkai(UDP &udp) {
    //     data[7] = 0;  // サーボ
    //     data[1] = 50; // ポンプ
    //     udp.send(data);
    //     std::cout << "フォーク展開" << std::endl;
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     folk_state = true;
    // }

    static void munagi_pickup_action(UDP &udp) {
        data[1] = 50; // ポンプ
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        data[8] = 90; // サーボ
        udp.send(data);
        std::cout << "ムナギ回収" << std::endl;
        folk_init_state = false;
    }

    static void hashira_pickup_action(UDP &udp) {
        data[7] = 0; // サーボ
        data[1] = 50;
        udp.send(data);
        std::cout << "柱回収" << std::endl;
        folk_init_state = false;
    }
};

// bool Action::folk_state = false; // フォークの状態初期化
bool Action::folk_init_state = false; // フォークが初期化されているか保存

class PS4_Listener : public rclcpp::Node {
public:
    PS4_Listener(const std::string &ip, int port)
        : Node("nhk10_natsu"), udp_(ip, port) {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "NATSUROBO2010 initialized with IP: %s, Port: %d", ip.c_str(),
                    port);
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
        bool TRIANGLE = msg->buttons[2];
        bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        bool L1 = msg->buttons[4];
        // bool R1 = msg->buttons[5];

        float L2 = (-1 * msg->axes[2] + 1) / 2;
        // float R2 = (-1 * msg->axes[5] + 1) / 2;

        // bool SHARE = msg->buttons[8];
        //  bool OPTION = msg->buttons[9];
        bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        static bool last_circle = false; // 前回の状態を保持する static 変数
        static bool last_triangle = false;
        static bool last_square = false;
        static bool last_up = false;
        static bool last_down = false;

        // ラッチstatic 変数（初期状態は OFF とする）
        static bool circle_latch = false;
        static bool triangle_latch = false;
        static bool square_latch = false;
        static bool up_latch = false;
        static bool down_latch = false;
        static int l1_mode = 0;
        static bool last_l1 = false; // L1ボタンの前回状態

        data[0] = MC_PRINTF; // マイコン側のprintfを無効化・有効化(0 or 1)

        if (!init_all_state) {
            Action::all_init(udp_);
            init_all_state = true;
        }

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
        // ラッチ(ボタンで変更)
        if (CIRCLE && !last_circle) {
            circle_latch = !circle_latch;
        }
        if (TRIANGLE && !last_triangle) {
            triangle_latch = !triangle_latch;
        }
        if (SQUARE && !last_square) {
            square_latch = !square_latch;
        }
        if (L1 && !last_l1) {
            l1_mode = (l1_mode + 1) % 4;
        }
        if (UP && !last_up) {
            up_latch = !up_latch;
        }
        if (DOWN && !last_down) {
            down_latch = !down_latch;
        }

        // ラッチのボタンとモードの指定
        last_circle = CIRCLE;
        last_triangle = TRIANGLE;
        last_square = SQUARE;
        last_up = UP;
        last_down = DOWN;
        last_l1 = L1;
        SHOOTMODE = l1_mode;

        int shoot_mode = 0;

        if (CIRCLE && init_all_state) {
            Action::munagi_pickup_action(udp_);
        }

        if (TRIANGLE && init_all_state) {
            Action::hashira_pickup_action(udp_);
        }

        if (CROSS) {
            Action::folk_init(udp_);
        }

        if (!square_latch) {
            data[9] = 0;
            shoot_enable = false; // シュート機構を無効化
        } else if (square_latch) {
            data[9] = 90;
            shoot_enable = true; // シュート機構を有効化
        }

        if (L2 > 0.5 && shoot_enable) {
            data[9] = 180;
        }

        // if (UP) {
        //     data[4] = -z_speed;
        //     udp_.send(data);
        //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //     data[4] = 0;
        // }

        // if (DOWN) {
        //     data[4] = z_speed;
        //     udp_.send(data);
        //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //     data[4] = 0;
        // }

        if (UP) {
            data[4] = -z_speed; // 押している間だけ上方向
        } else if (DOWN) {
            data[4] = z_speed; // 押している間だけ下方向
        } else {
            data[4] = 0; // どちらも押していなければ停止
        }

        if (L1 && !last_l1) {
            shoot_mode++;
        }

        std::cout << SHOOTMODE << std::endl;

        if (SHOOTMODE == 0) {
            speed_shoot = 0;
        } else if (SHOOTMODE == 1) {
            speed_shoot = 50;
        } else if (SHOOTMODE == 2) {
            speed_shoot = 60;
        } else if (SHOOTMODE == 3) {
            speed_shoot = 70;
        }
        data[3] = speed_shoot; // 射出機構の速度設定

        // if (OPTION) {
        //     Ball_Action::tester(udp_);
        // }
        // デバッグ用（for文でcoutするとカクつく）
        //  std::cout << data[0] << ", " << data[1] << ", " << data[2] << ", " << data[3] << ", ";
        //  std::cout << data[4] << ", " << data[5] << ", " << data[6] << ", " << data[7] << ", ";
        //  std::cout << data[8] << ", " << data[9] << ", " << data[10] << ", " << data[11] << ", ";
        //  std::cout << data[12] << ", " << data[13] << ", " << data[14] << ", " << data[15] << ", ";
        //  std::cout << data[16] << ", " << data[17] << ", " << data[18] << std::endl;
        //  std::cout << data[11] << std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));

        udp_.send(data);
        // std::cout << circle_latch << std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    UDP udp_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet RRST NATSUROBO";
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
    auto ps4_listener = std::make_shared<PS4_Listener>(IP_NATSU, PORT_NATSU);
    exec.add_node(ps4_listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
