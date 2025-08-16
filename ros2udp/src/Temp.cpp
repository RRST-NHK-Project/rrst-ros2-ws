/*
RRST-NHK-Project 2025
テスト用（自由に書き換えて良い）
*/

/*
＊テスト内容＊
・UDPクラスの通信容量削減に向けて新クラスの開発
・新クラスの動作確認
*/

// 標準
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>

// 自作クラス
#include "include/IP.hpp"
#include "include/UDP_Lite.hpp"

#define MC_PRINTF 0 // マイコン側のprintfを無効化・有効化(0 or 1)

// スティックのデッドゾーン
#define DEADZONE_L 0.3
#define DEADZONE_R 0.3

// 定数k
#define k 0.05

// PIDパラメータ（チューニングが必要）
#define speed_Kp 0.3  // 速度Pゲイン
#define speed_Ki 0.05 // 速度Iゲイン
#define speed_Kd 0.05 // 速度Dゲイン

#define speed_limit 30
#define deg_limit 360
#define DPAD_SPEED 30 // 方向パッド入力時の目標速度

bool CHANGEMODE = false;
bool REVERSEMODE = false;
bool YAWMODE = false;

// グローバル変数（角度一覧）
int deg = 0;
int previous_deg = 0;
int truedeg = 0;
int desired_deg = 0;
int measured_deg = 0;

// 速度
int wheelspeed = 25;
int yawspeed = -12;
int yawspeed_auto = 20;

// サーボの組み付け時のズレを補正（度数法）
int SERVO1_CAL = 10;
int SERVO2_CAL = 8;
int SERVO3_CAL = 23;
int SERVO4_CAL = 20;

// 最近傍点距離の格納
float min_distance = 0;
bool front_cleared = false;
float front_cleared_distance = 0.8; // 障害物検知のしきい値（要調整）

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

// 自動化クラス
class Automation {
public:
    // 自動ターン（決め打ち）
    // 反転モードと同時に180度回転する
    static void auto_turn(UDP_Lite &udp) {
        std::cout << "自動ターン開始" << std::endl;
        data[7] = 180 + SERVO1_CAL;
        data[8] = 90 + SERVO2_CAL;
        data[9] = 90 + SERVO3_CAL;
        data[10] = 180 + SERVO4_CAL;
        data[1] = -yawspeed_auto;
        data[2] = yawspeed_auto;
        data[3] = -yawspeed_auto;
        data[4] = yawspeed_auto;
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "自動ターン終了" << std::endl;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        udp.send(data);
    }
};

class PS4_Listener : public rclcpp::Node {
public:
    PS4_Listener(const std::string &ip, int port)
        : Node("nhk25_mr_sd"), udp_(ip, port) {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy0", 10,
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
        float RS_X = -1 * msg->axes[3];
        // float RS_Y = msg->axes[4];

        bool CROSS = msg->buttons[0];
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

        bool SHARE = msg->buttons[8];
        bool OPTION = msg->buttons[9];
        bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        bool R3 = msg->buttons[12];

        static bool last_option = false; // 前回の状態を保持する static 変数
        static bool last_share = false;

        // OPTION のラッチ状態を保持する static 変数（初期状態は OFF とする）
        static bool option_latch = false;
        static bool share_latch = false;

        static bool last_R3 = false;
        static bool R3_latch = false;

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

        float rad = atan2(LS_Y, LS_X);
        deg = rad * 180 / M_PI;
        if (OPTION && !last_option) {
            option_latch = !option_latch;
        }
        if (SHARE && !last_share) {
            share_latch = !share_latch;
            // Automation::auto_turn(udp_);
        }
        if (R3 && !last_R3) {
            R3_latch = !R3_latch;
        }

        if (front_cleared == false) {
            // std::cout << "障害物検知！" << std::endl;
            data[13] = 1;
            udp_.send(data);
        } else {
            data[13] = 0; // 障害物がない場合は0
        }

        // ！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
        // もとの移動方法！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
        // ！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！

        last_share = SHARE;
        REVERSEMODE = share_latch;
        last_option = OPTION;
        CHANGEMODE = option_latch;
        last_R3 = R3;
        YAWMODE = R3_latch;

        // XY座標での正しい角度truedeg

        if (REVERSEMODE == 0) {
            data[11] = 0; // テープLED消灯
            data[12] = 1;
            truedeg = deg;
            if ((0 <= truedeg) && (truedeg <= 180)) {
                truedeg = truedeg;
            }
            if ((-180 <= truedeg) && (truedeg <= 0)) {
                truedeg = -truedeg + 360;
            }

            // ！！！！！最重要！！！！！
            //  XY座標での９０度の位置に１３５度を変換して計算
            if ((-180 <= deg) && (deg <= -135)) {
                deg = -deg - 135;
            } else {
                deg = 225 - deg;
            }

            // deadzone追加
            if ((fabs(LS_X) <= DEADZONE_R) && (fabs(LS_Y) <= DEADZONE_R) &&
                (fabs(RS_X) <= DEADZONE_L)) {
                deg = 135;
                data[1] = 0;
                data[2] = 0;
                data[3] = 0;
                data[4] = 0;
                data[7] = deg + SERVO1_CAL;
                data[8] = deg + SERVO2_CAL;
                data[9] = deg + SERVO3_CAL;
                data[10] = deg + SERVO4_CAL;
            }

            data[1] = -wheelspeed * R2;
            data[2] = -wheelspeed * R2;
            data[3] = -wheelspeed * R2;
            data[4] = -wheelspeed * R2;
            data[7] = deg + SERVO1_CAL;
            data[8] = deg + SERVO2_CAL;
            data[9] = deg + SERVO3_CAL;
            data[10] = deg + SERVO4_CAL;

            if (LEFT) {
                deg = 45;
                data[1] = -wheelspeed * R2;
                data[2] = -wheelspeed * R2;
                data[3] = -wheelspeed * R2;
                data[4] = -wheelspeed * R2;
            }
            if (RIGHT) {
                deg = 45;
                data[1] = wheelspeed * R2;
                data[2] = wheelspeed * R2;
                data[3] = wheelspeed * R2;
                data[4] = wheelspeed * R2;
            }
            if (UP) {
                deg = 135;
                data[1] = -wheelspeed * R2;
                data[2] = -wheelspeed * R2;
                data[3] = -wheelspeed * R2;
                data[4] = -wheelspeed * R2;
            }
            if (DOWN) {
                deg = 135;
                data[1] = wheelspeed * R2;
                data[2] = wheelspeed * R2;
                data[3] = wheelspeed * R2;
                data[4] = wheelspeed * R2;
            }

            // 独ステが扱えない範囲の変換
            if ((270 < deg) && (deg < 360)) {
                deg = deg - 180;
                data[1] = wheelspeed * R2;
                data[2] = wheelspeed * R2;
                data[3] = wheelspeed * R2;
                data[4] = wheelspeed * R2;
                data[7] = deg + SERVO1_CAL;
                data[8] = deg + SERVO2_CAL;
                data[9] = deg + SERVO3_CAL;
                data[10] = deg + SERVO4_CAL;
            }
            // 角度だけYAW
            if (R3_latch == 0) {
                data[7] = deg + SERVO1_CAL;
                data[8] = deg + SERVO2_CAL;
                data[9] = deg + SERVO3_CAL;
                data[10] = deg + SERVO4_CAL;
            }
            if (R3_latch == 1) {
                data[7] = 180 + SERVO1_CAL;
                data[8] = 90 + SERVO2_CAL;
                data[9] = 90 + SERVO3_CAL;
                data[10] = 180 + SERVO4_CAL;
            }
            // 時計回りYAW回転
            if (RS_X < 0 && fabs(RS_X) >= DEADZONE_R) {
                data[7] = 180 + SERVO1_CAL;
                data[8] = 90 + SERVO2_CAL;
                data[9] = 90 + SERVO3_CAL;
                data[10] = 180 + SERVO4_CAL;
                data[1] = -yawspeed;
                data[2] = yawspeed;
                data[3] = -yawspeed;
                data[4] = yawspeed;
            }
            // 半時計回りYAW回転
            if (0 < RS_X && fabs(RS_X) >= DEADZONE_R) {
                data[7] = 180 + SERVO1_CAL;
                data[8] = 90 + SERVO2_CAL;
                data[9] = 90 + SERVO3_CAL;
                data[10] = 180 + SERVO4_CAL;
                data[1] = yawspeed;
                data[2] = -yawspeed;
                data[3] = yawspeed;
                data[4] = -yawspeed;
            }
        }
        // 反転モード
        //
        if (REVERSEMODE == 1) {
            data[11] = 1; // テープLED点灯
            data[12] = 0;
            truedeg = deg;
            if ((0 <= truedeg) && (truedeg <= 180)) {
                truedeg = truedeg;
            }
            if ((-180 <= truedeg) && (truedeg <= 0)) {
                truedeg = -truedeg + 360;
            }

            // ！！！！！最重要！！！！！
            //  XY座標での９０度の位置に１３５度を変換して計算
            if ((-180 <= deg) && (deg <= -135)) {
                deg = -deg - 135;
            } else {
                deg = 225 - deg;
            }

            // deadzone追加
            if ((fabs(LS_X) <= DEADZONE_R) && (fabs(LS_Y) <= DEADZONE_R) &&
                (fabs(RS_X) <= DEADZONE_L)) {
                deg = 135;
                data[1] = 0;
                data[2] = 0;
                data[3] = 0;
                data[4] = 0;
                data[7] = deg + SERVO1_CAL;
                data[8] = deg + SERVO2_CAL;
                data[9] = deg + SERVO3_CAL;
                data[10] = deg + SERVO4_CAL;
            }

            data[1] = wheelspeed * R2;
            data[2] = wheelspeed * R2;
            data[3] = wheelspeed * R2;
            data[4] = wheelspeed * R2;
            data[7] = deg + SERVO1_CAL;
            data[8] = deg + SERVO2_CAL;
            data[9] = deg + SERVO3_CAL;
            data[10] = deg + SERVO4_CAL;

            if (LEFT) {
                deg = 45;
                data[1] = wheelspeed * R2;
                data[2] = wheelspeed * R2;
                data[3] = wheelspeed * R2;
                data[4] = wheelspeed * R2;
            }
            if (RIGHT) {
                deg = 45;
                data[1] = -wheelspeed * R2;
                data[2] = -wheelspeed * R2;
                data[3] = -wheelspeed * R2;
                data[4] = -wheelspeed * R2;
            }
            if (UP) {
                deg = 135;
                data[1] = wheelspeed * R2;
                data[2] = wheelspeed * R2;
                data[3] = wheelspeed * R2;
                data[4] = wheelspeed * R2;
            }
            if (DOWN) {
                deg = 135;
                data[1] = -wheelspeed * R2;
                data[2] = -wheelspeed * R2;
                data[3] = -wheelspeed * R2;
                data[4] = -wheelspeed * R2;
            }

            // 射出直前にサーボを直角に向けストップ
            if (CROSS) {
                data[1] = 0;
                data[2] = 0;
                data[3] = 0;
                data[4] = 0;
                data[5] = 0;
                data[6] = 0;
                data[7] = 45 + SERVO1_CAL;
                data[8] = 45 + SERVO2_CAL;
                data[9] = 45 + SERVO3_CAL;
                data[10] = 45 + SERVO4_CAL;
            }

            // 独ステが扱えない範囲の変換
            if ((270 < deg) && (deg < 360)) {
                deg = deg - 180;
                data[1] = -wheelspeed * R2;
                data[2] = -wheelspeed * R2;
                data[3] = -wheelspeed * R2;
                data[4] = -wheelspeed * R2;
                data[7] = deg + SERVO1_CAL;
                data[8] = deg + SERVO2_CAL;
                data[9] = deg + SERVO3_CAL;
                data[10] = deg + SERVO4_CAL;
            }
            // 角度だけYAW
            if (R3_latch == 0) {
                data[7] = deg + SERVO1_CAL;
                data[8] = deg + SERVO2_CAL;
                data[9] = deg + SERVO3_CAL;
                data[10] = deg + SERVO4_CAL;
            }
            if (R3_latch == 1) {
                data[7] = 180 + SERVO1_CAL;
                data[8] = 90 + SERVO2_CAL;
                data[9] = 90 + SERVO3_CAL;
                data[10] = 180 + SERVO4_CAL;
            }
            // 時計回りYAW回転
            if (RS_X < 0 && fabs(RS_X) >= DEADZONE_R) {
                data[7] = 180 + SERVO1_CAL;
                data[8] = 90 + SERVO2_CAL;
                data[9] = 90 + SERVO3_CAL;
                data[10] = 180 + SERVO4_CAL;
                data[1] = -yawspeed;
                data[2] = yawspeed;
                data[3] = -yawspeed;
                data[4] = yawspeed;
            }
            // 半時計回りYAW回転
            if (0 < RS_X && fabs(RS_X) >= DEADZONE_R) {
                data[7] = 180 + SERVO1_CAL;
                data[8] = 90 + SERVO2_CAL;
                data[9] = 90 + SERVO3_CAL;
                data[10] = 180 + SERVO4_CAL;
                data[1] = yawspeed;
                data[2] = -yawspeed;
                data[3] = yawspeed;
                data[4] = -yawspeed;
            }
        }

        if (SHARE) {
            // Automation::auto_turn(udp_);
        }
        // std::cout << REVERSEMODE << std::endl;

        // デバッグ用（for文でcoutするとカクつく）
        // std::cout << data[0] << ", " << data[1] << ", " << data[2] << ", " << data[3] << ", ";
        // std::cout << data[4] << ", " << data[5] << ", " << data[6] << ", " << data[7] << ", ";
        // std::cout << data[8] << ", " << data[9] << ", " << data[10] << ", " << data[11] << ", ";
        // std::cout << data[12] << ", " << data[13] << ", " << data[14] << ", " << data[15] << ", ";
        // std::cout << data[16] << ", " << data[17] << ", " << data[18] << std::endl;
        // std::cout << data[11] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // std::cout << REVERSEMODE << std::endl;
        udp_.send(data);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    UDP_Lite udp_;
};

class Servo_Deg_Publisher : public rclcpp::Node {
public:
    Servo_Deg_Publisher() : Node("mr_servo_deg_publisher") {
        // Publisherの作成
        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "mr_servo_deg", 10);

        // タイマーを使って定期的にメッセージをpublish
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&Servo_Deg_Publisher::publish_message, this));
    }

private:
    void publish_message() {
        auto message = std_msgs::msg::Int32MultiArray();
        message.data = {data[5], data[6], data[7], data[8]};

        // RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
        publisher_->publish(message); // メッセージをpublish
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

class Params_Listener : public rclcpp::Node {
public:
    Params_Listener() : Node("nr25_mr_servo_cal_listener") {
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "mr_servo_cal", 10,
            std::bind(&Params_Listener::params_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "MR Servo Calibrator Listener");
    }

private:
    void params_listener_callback(
        const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        SERVO1_CAL = msg->data[0];
        SERVO2_CAL = msg->data[1];
        SERVO3_CAL = msg->data[2];
        SERVO4_CAL = msg->data[3];
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

// LD19（LiDAR）から取得した最近傍点までの距離を受信するクラス
class LD19_Listener : public rclcpp::Node {
public:
    LD19_Listener() // ここがコンストラクタ！
        : Node("nhk25_dr_ld19") {
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "nearest_point", 10,
            std::bind(&LD19_Listener::ld19_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "NHK2025 LD19 Listener");
    }

private:
    void ld19_listener_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        min_distance = msg->data[0];
        // std::cout << min_distance << std::endl;
        // 障害物の有無（しきい値は要調整）
        if (min_distance < front_cleared_distance) {
            front_cleared = false;
        } else {
            front_cleared = true;
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet MR SwerveDrive";
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

    rclcpp::executors::MultiThreadedExecutor exec;
    auto ps4_listener = std::make_shared<PS4_Listener>(IP_TEST, PORT_TEST);
    auto servo_deg_publisher = std::make_shared<Servo_Deg_Publisher>();
    auto params_listener = std::make_shared<Params_Listener>();
    auto ld19_listener = std::make_shared<LD19_Listener>();
    exec.add_node(ps4_listener);
    exec.add_node(servo_deg_publisher);
    exec.add_node(params_listener);
    exec.add_node(ld19_listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}