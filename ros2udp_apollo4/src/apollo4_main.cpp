/*
RRST-NHK-Project 2025
キャチロボ：apollo4
メインアーム＆メインハンド
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
#include "include/UDP.hpp"

#define MC_PRINTF 0 // マイコン側のprintfを無効化・有効化(0 or 1)

// L2,R2のデッドゾーン
#define DEADZONE_L2 0.3
#define DEADZONE_R2 0.3

//R1,TRIANGLEの切り替えに必要
int R1_count = 0;           // R1が押された回数
int TRIANGLE_count = 0;     // TRIANGLEが押された回数
bool prev_R1 = false;       // 前回のR1の状態
bool prev_TRIANGLE = false;       // 前回のTRIANGLEの状態

//モード切り替え
bool CHANGEMODE = false;

// 速度
int wheelspeed = 25;
int absorbspeed = 30;

//サーボ
int servo_angle = 90;        // 現在の角度（初期は90度）
int servo_step = 1;          // 1回の更新で動く角度

// サーボの組み付け時のズレを補正（度数法）
int SERVO1_CAL = 10;
int SERVO2_CAL = 8;
int SERVO3_CAL = 23;
int SERVO4_CAL = 20;
int SERVO5_CAL = 10;
int SERVO6_CAL = 10;


std::vector<int16_t> data(19, 0); // マイコンに送信される配列"data"
/*
マイコンに送信される配列"data"
debug: マイコンのprintfを有効化, MD: モータードライバー, TR: トランジスタ ,ENC: エンコーダ
| data[n] | 詳細 | 範囲 |
| ---- | ---- | ---- |
| data[0] | debug | 0 or 1 |
MD1,2,3でアームを制御(R,θ,Z)軸,MD4,5でDCを５個ずつ制御
| data[1] | MD1 | -100 ~ 100 |
| data[2] | MD2 | -100 ~ 100 |
| data[3] | MD3 | -100 ~ 100 |
| data[4] | MD4 | -100 ~ 100 |
| data[5] | MD5 | -100 ~ 100 |
| data[6] | MD6 | -100 ~ 100 |
| data[7] | MD7 | -100 ~ 100 |
| data[8] | MD8 | -100 ~ 100 |
Servo1~5がMG90D,Servo6がDS3218
| data[9] | Servo1 | 0 ~ 270 |
| data[10] | Servo2 | 0 ~ 270 |
| data[11] | Servo3 | 0 ~ 270 |
| data[12] | Servo4 | 0 ~ 270 |
| data[13] | Servo5 | 0 ~ 270 |
| data[14] | Servo6 | 0 ~ 270 |

| data[15] | ENC1 | 0 or 1|
| data[16] | ENC2 | 0 or 1|
| data[17] | ENC3 | 0 or 1|
| data[18] | ENC4 | 0 or 1|
*/


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
        //float LS_X = -1 * msg->axes[0];
        //float LS_Y = msg->axes[1];
        //float RS_X = -1 * msg->axes[3];
        // float RS_Y = msg->axes[4];

        bool CROSS = msg->buttons[0];
        bool CIRCLE = msg->buttons[1];
        bool TRIANGLE = msg->buttons[2];
        bool SQUARE = msg->buttons[3];

        bool LEFT = msg->axes[6] == 1.0;
        bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        bool L1 = msg->buttons[4];
        bool R1 = msg->buttons[5];

        float L2 = (-1 * msg->axes[2] + 1) / 2;
        float R2 = (-1 * msg->axes[5] + 1) / 2;

        bool SHARE = msg->buttons[8];
        bool OPTION = msg->buttons[9];
        bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        bool R3 = msg->buttons[12];

        //L2でゼロイチ制御するためdeadzone追加
        if(L2 <= DEADZONE_L2){
            L2 = 0; 
        }
        // 前回の状態を保持する static 変数
        static bool last_PS = false;

        // PS のラッチ状態を保持する static 変数（初期状態は OFF とする）
        static bool PS_latch = false;

        data[0] = MC_PRINTF; // マイコン側のprintfを無効化・有効化(0 or 1)

        if (PS && !last_PS) {
            PS_latch = !PS_latch;
        }
        last_PS = PS;
        CHANGEMODE = PS_latch;
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
        // RθZの操作!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // R軸の操作
        if (UP) {       // 正転
            data[1] = -wheelspeed;
        }
        if (DOWN) {     // 逆転
            data[1] = wheelspeed;
        }
        // θ軸の操作
        if (LEFT) {     // 正転
            data[2] = -wheelspeed;
        }
        if (RIGHT) {    // 逆転
            data[2] = wheelspeed;
        }
        // Z軸の操作
        if(L1) {         // 正転
            data[3] = -wheelspeed; 
        }
        if(L2>0) {       // 逆転
            data[3] = wheelspeed;
        }

        // 展開指変更!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // 0 → 1 に変化したときだけカウントしR1_countが加算されていく
        if (R1 && !prev_R1) {   // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
            R1_count++;
        }
        if(R1_count %6 == 0) {  // 垂直の状態
            data[9] = 90 + SERVO1_CAL;
            data[10] = 90 + SERVO2_CAL;
            data[11] = 90 + SERVO3_CAL;
            data[12] = 90 + SERVO4_CAL;
            data[13] = 90 + SERVO5_CAL;
        }
        if(R1_count %6 == 1) {  // 取る状態1
            data[9] = 45 + SERVO1_CAL;
            data[10] = 100 + SERVO2_CAL;
            data[11] = 100 + SERVO3_CAL;
            data[12] = 100 + SERVO4_CAL;
            data[13] = 100 + SERVO5_CAL;
        }
        if(R1_count %6 == 2) {  // 取る状態2
            data[9] = 100 + SERVO1_CAL;
            data[10] = 45 + SERVO2_CAL;
            data[11] = 100 + SERVO3_CAL;
            data[12] = 100 + SERVO4_CAL;
            data[13] = 100 + SERVO5_CAL;
        }
        if(R1_count %6 == 3) {  // 取る状態3
            data[9] = 100 + SERVO1_CAL;
            data[10] = 100 + SERVO2_CAL;
            data[11] = 45 + SERVO3_CAL;
            data[12] = 100 + SERVO4_CAL;
            data[13] = 100 + SERVO5_CAL;
        }
        if(R1_count %6 == 4){   // 取る状態4
            data[9] = 100 + SERVO1_CAL;
            data[10] = 100 + SERVO2_CAL;
            data[11] = 100 + SERVO3_CAL;
            data[12] = 45 + SERVO4_CAL;
            data[13] = 100 + SERVO5_CAL;
        }
        if(R1_count %6 == 5){   // 取る状態5
            data[9] = 100 + SERVO1_CAL;
            data[10] = 100 + SERVO2_CAL;
            data[11] = 100 + SERVO3_CAL;
            data[12] = 100 + SERVO4_CAL;
            data[13] = 45 + SERVO5_CAL;
        }

        // サーボの手先角度操作!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // ボタン入力に応じて角度を変化
        // DS3218proなので270度までいけます
        if (SQUARE && !CIRCLE){
        // 徐々に正転
        servo_angle += servo_step;
        if (servo_angle > 270) {// 上限で止める
            servo_angle = 270;
            }
        } else if (CIRCLE && !SQUARE){
        // 徐々に逆転
        servo_angle -= servo_step;
        if (servo_angle < 0) {// 下限で止める
            servo_angle = 0;
            }
        }
        // サーボ指令
        data[15] = servo_angle + SERVO6_CAL;

        //吸着モード切り替え!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // 0 → 1 に変化したときだけカウントしTRIANGLE_countが加算されていく
        if (TRIANGLE && !prev_TRIANGLE) {   // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
            TRIANGLE_count++;
        }
        if(TRIANGLE_count %3 == 0) {  // 全OFF(Cモード)
            data[4] = 0;
            data[5] = 0;
            if(CROSS){                // 吸着割り込み(☓のときAモードにする)
                data[4] = absorbspeed;
                data[5] = absorbspeed;
            }
        }
        if(TRIANGLE_count %3 == 1) {  // 全ON(Aモード)
            data[4] = absorbspeed;
            data[5] = absorbspeed;
        }
        if(TRIANGLE_count %3 == 2) {  // 上ON/下OFF(Bモード)
            data[4] = absorbspeed;
            data[5] = 0;
            if(CROSS){                // 吸着割り込み(☓のときAモードにする)
                data[4] = absorbspeed;
                data[5] = absorbspeed;
            }
        }

        // if(CHANGEMODE == 0){
        //     data[9] = 90 + SERVO1_CAL;
        //     data[10] = 90 + SERVO2_CAL;
        //     data[11] = 90 + SERVO3_CAL;
        //     data[12] = 90 + SERVO4_CAL;
        //     data[13] = 90 + SERVO5_CAL;
        // }
        // if(CHANGEMODE == 1){
            
        // }


        // 状態を更新
        prev_R1 = R1; 
        prev_TRIANGLE = TRIANGLE;



        // デバッグ用（for文でcoutするとカクつく）
        // std::cout << data[0] << ", " << data[1] << ", " << data[2] << ", " << data[3] << ", ";
        // std::cout << data[4] << ", " << data[5] << ", " << data[6] << ", " << data[7] << ", ";
        // std::cout << data[8] << ", " << data[9] << ", " << data[10] << ", " << data[11] << ", ";
        // std::cout << data[12] << ", " << data[13] << ", " << data[14] << ", " << data[15] << ", ";
        // std::cout << data[16] << ", " << data[17] << ", " << data[18] << std::endl;
        std::cout << data[4] << ", " << data[5] << std::endl; //吸着できているか確認する用

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        udp_.send(data);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    UDP udp_;
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
        message.data = {data[9], data[10], data[11], data[12], data[13], data[14]};

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
        SERVO5_CAL = msg->data[4];
        SERVO6_CAL = msg->data[5];
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
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
    auto ps4_listener = std::make_shared<PS4_Listener>(IP_MR_SD, PORT_MR_SD);
    auto servo_deg_publisher = std::make_shared<Servo_Deg_Publisher>();
    auto params_listener = std::make_shared<Params_Listener>();
    exec.add_node(ps4_listener);
    exec.add_node(servo_deg_publisher);
    exec.add_node(params_listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}