/*
RRST-NHK-Project 2025
キャチロボ：apollo4
シューティングアーム＆シューティングハンド
*/

// 標準ライブラリ
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

// --- グローバル設定 ---
// モーターの基本速度 (-100 ~ 100の範囲を想定)
constexpr int MOTOR_SPEED = 80;
// L2/R2トリガーが反応しない領域（誤操作防止）
constexpr float TRIGGER_DEADZONE = 0.7;

//L3,R3の切り替えに必要
int R_count = 0;           // R1、R2が押された回数

// 状態を保持するための変数
bool control_active_ = false; // 制御が有効か (PSボタンでトグル)
bool gripper_closed_ = false; // グリッパーが閉じているか (〇ボタンでトグル)

// ボタンが押された瞬間を検知するための変数
bool last_ps_state_ = false;
bool last_circle_state_ = false;

//R1,R2,L1,L2の切り替えに必要
bool prev_R1 = false;       //前回のR1の状態
bool prev_R2 = false;       //前回のR2の状態
bool prev_R3 = false;       //前回のR3の状態
bool prev_L1 = false;       //前回のL1の状態
bool prev_L2 = false;       //前回のL2の状態
bool prev_L3 = false;       //前回のL3の状態
bool prev_LEFT = false;     //前回のLEFTの状態
bool prev_RIGHT = false;    //前回のRIGHTの状態
bool prev_UP = false;       //前回のUPの状態
bool prev_DOWN = false;     //前回のDOWNの状態
bool prev_SQUARE = false;   //前回のSQUAREの状態
bool prev_CIRCLE = false;   //前回のCIRCLEの状態
bool prev_TRIANGLE = false; //前回のTRIANGLEの状態
bool prev_CROSS = false;   //前回のTCIRCLEの状態

//モード切り替え
bool CHANGEMODE = false;
bool PtoPMODE = false;

//サーボ
int SERVO1_angle = 70;      //サーボ1
int SERVO2_angle = 70;      //サーボ2
int SERVO3_angle = 70;      //サーボ3
int SERVO4_angle = 0;      //サーボ4
int SERVO5_angle = 5;      //サーボ5



// マイコンに送信されるデータ配列
// [θ1, θ2, θ3, θ4, θ5, グリッパー] の指令値を格納
std::vector<int16_t> data(23, 0);
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
| data[11] | Servo5 | 0 ~ 270 |
| data[12] | TR1 | 0 or 1|
| data[13] | TR2 | 0 or 1|
| data[14] | TR3 | 0 or 1|
| data[15] | TR4 | 0 or 1|
| data[16] | TR5 | 0 or 1|
| data[17] | TR6 | 0 or 1|
| data[18] | TR7 | 0 or 1|
| data[19] | TR8 | 0 or 1|
*/

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
| data[11] | Servo5 | 0 ~ 270 |
| data[12] | Servo6 | 0 ~ 270 |
| data[13] | Servo7 | 0 ~ 270 | 
| data[14] | Servo8 | 0 ~ 270 |
| data[15] | TR1 | 0 or 1|
| data[16] | TR2 | 0 or 1|
| data[17] | TR3 | 0 or 1|
| data[18] | TR4 | 0 or 1|
| data[19] | TR5 | 0 or 1|
| data[20] | TR6 | 0 or 1|
| data[21] | TR7 | 0 or 1|
| data[12] | TR8 | 0 or 1|
*/

// 各機構のシーケンスを格納するクラス
class Action {
public:
    // 事故防止のため、射出機構の展開状況を保存
    static bool reload_state;
    static bool shoot_state;

    // case1_blue
    static void case1_blue_action(UDP &udp) {
        std::cout << "<case1_blueシーケンス開始>" << std::endl;
        SERVO1_angle = 1;
        SERVO2_angle = 1;
        SERVO3_angle = 1;
        SERVO4_angle = 1;
        SERVO5_angle = 1;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        udp.send(data);
        std::cout << data[7] << data[8] << data[9] << data[10] << data[11] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        SERVO1_angle = 1;
        SERVO2_angle = 1;
        SERVO3_angle = 1;
        SERVO4_angle = 1;
        SERVO5_angle = 1;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        udp.send(data);
        std::cout << "case1_blue終了" << std::endl;
    }

    // case2_blue
    static void case2_blue_action(UDP &udp) {
        std::cout << "<case2_blueシーケンス開始>" << std::endl;
        SERVO1_angle = 2;
        SERVO2_angle = 2;
        SERVO3_angle = 2;
        SERVO4_angle = 2;
        SERVO5_angle = 2;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        udp.send(data);
        std::cout << data[7] << data[8] << data[9] << data[10] << data[11] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        SERVO1_angle = 2;
        SERVO2_angle = 2;
        SERVO3_angle = 2;
        SERVO4_angle = 2;
        SERVO5_angle = 2;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        udp.send(data);
        std::cout << "case2_blue終了" << std::endl;
    }

    // case3_blue
    static void case3_blue_action(UDP &udp) {
        std::cout << "<case3_blueシーケンス開始>" << std::endl;
        SERVO1_angle = 3;
        SERVO2_angle = 3;
        SERVO3_angle = 3;
        SERVO4_angle = 3;
        SERVO5_angle = 3;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        udp.send(data);
        std::cout << data[7] << data[8] << data[9] << data[10] << data[11] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        SERVO1_angle = 3;
        SERVO2_angle = 3;
        SERVO3_angle = 3;
        SERVO4_angle = 3;
        SERVO5_angle = 3;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        udp.send(data);
        std::cout << "case3_blue終了" << std::endl;
    }

    // case4_blue
    static void case4_blue_action(UDP &udp) {
        std::cout << "<case4_blueシーケンス開始>" << std::endl;
        SERVO1_angle = 4;
        SERVO2_angle = 4;
        SERVO3_angle = 4;
        SERVO4_angle = 4;
        SERVO5_angle = 4;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        udp.send(data);
        std::cout << data[7] << data[8] << data[9] << data[10] << data[11] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        SERVO1_angle = 4;
        SERVO2_angle = 4;
        SERVO3_angle = 4;
        SERVO4_angle = 4;
        SERVO5_angle = 4;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        udp.send(data);
        std::cout << "case4_blue終了" << std::endl;
    }

    // case5_blue
    static void case5_blue_action(UDP &udp) {
        std::cout << "<case5_blueシーケンス開始>" << std::endl;
        SERVO1_angle = 5;
        SERVO2_angle = 5;
        SERVO3_angle = 5;
        SERVO4_angle = 5;
        SERVO5_angle = 5;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        udp.send(data);
        std::cout << data[7] << data[8] << data[9] << data[10] << data[11] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        SERVO1_angle = 5;
        SERVO2_angle = 5;
        SERVO3_angle = 5;
        SERVO4_angle = 5;
        SERVO5_angle = 5;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        udp.send(data);
        std::cout << "case5_blue終了" << std::endl;
    }

    // case6_blue
    static void case6_blue_action(UDP &udp) {
        std::cout << "<case6_blueシーケンス開始>" << std::endl;
        SERVO1_angle = 6;
        SERVO2_angle = 6;
        SERVO3_angle = 6;
        SERVO4_angle = 6;
        SERVO5_angle = 6;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        udp.send(data);
        std::cout << data[7] << data[8] << data[9] << data[10] << data[11] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        SERVO1_angle = 6;
        SERVO2_angle = 6;
        SERVO3_angle = 6;
        SERVO4_angle = 6;
        SERVO5_angle = 6;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        udp.send(data);
        std::cout << "case6_blue終了" << std::endl;
    }

    // case7_blue
    static void case7_blue_action(UDP &udp) {
        std::cout << "<case7_blueシーケンス開始>" << std::endl;
        SERVO1_angle = 7;
        SERVO2_angle = 7;
        SERVO3_angle = 7;
        SERVO4_angle = 7;
        SERVO5_angle = 7;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        udp.send(data);
        std::cout << data[7] << data[8] << data[9] << data[10] << data[11] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        SERVO1_angle = 7;
        SERVO2_angle = 7;
        SERVO3_angle = 7;
        SERVO4_angle = 7;
        SERVO5_angle = 7;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        udp.send(data);
        std::cout << "case7_blue終了" << std::endl;
    }

    // case1_red
    static void case1_red_action(UDP &udp) {
        std::cout << "<case1_redシーケンス開始>" << std::endl;
        SERVO1_angle = 1;
        SERVO2_angle = 1;
        SERVO3_angle = 1;
        SERVO4_angle = 1;
        SERVO5_angle = 1;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        udp.send(data);
        std::cout << data[7] << data[8] << data[9] << data[10] << data[11] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        SERVO1_angle = 1;
        SERVO2_angle = 1;
        SERVO3_angle = 1;
        SERVO4_angle = 1;
        SERVO5_angle = 1;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        udp.send(data);
        std::cout << "case1_red終了" << std::endl;
    }

    // case2_red
    static void case2_red_action(UDP &udp) {
        std::cout << "<case2_redシーケンス開始>" << std::endl;
        SERVO1_angle = 2;
        SERVO2_angle = 2;
        SERVO3_angle = 2;
        SERVO4_angle = 2;
        SERVO5_angle = 2;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        udp.send(data);
        std::cout << data[7] << data[8] << data[9] << data[10] << data[11] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        SERVO1_angle = 2;
        SERVO2_angle = 2;
        SERVO3_angle = 2;
        SERVO4_angle = 2;
        SERVO5_angle = 2;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        udp.send(data);
        std::cout << "case2_red終了" << std::endl;
    }

    // case3_red
    static void case3_red_action(UDP &udp) {
        std::cout << "<case3_redシーケンス開始>" << std::endl;
        SERVO1_angle = 3;
        SERVO2_angle = 3;
        SERVO3_angle = 3;
        SERVO4_angle = 3;
        SERVO5_angle = 3;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        udp.send(data);
        std::cout << data[7] << data[8] << data[9] << data[10] << data[11] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        SERVO1_angle = 3;
        SERVO2_angle = 3;
        SERVO3_angle = 3;
        SERVO4_angle = 3;
        SERVO5_angle = 3;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        udp.send(data);
        std::cout << "case3_red終了" << std::endl;
    }

    // case4_red
    static void case4_red_action(UDP &udp) {
        std::cout << "<case4_redシーケンス開始>" << std::endl;
        SERVO1_angle = 4;
        SERVO2_angle = 4;
        SERVO3_angle = 4;
        SERVO4_angle = 4;
        SERVO5_angle = 4;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        udp.send(data);
        std::cout << data[7] << data[8] << data[9] << data[10] << data[11] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        SERVO1_angle = 4;
        SERVO2_angle = 4;
        SERVO3_angle = 4;
        SERVO4_angle = 4;
        SERVO5_angle = 4;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        udp.send(data);
        std::cout << "case4_red終了" << std::endl;
    }

    // case5_red
    static void case5_red_action(UDP &udp) {
        std::cout << "<case5_redシーケンス開始>" << std::endl;
        SERVO1_angle = 5;
        SERVO2_angle = 5;
        SERVO3_angle = 5;
        SERVO4_angle = 5;
        SERVO5_angle = 5;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        udp.send(data);
        std::cout << data[7] << data[8] << data[9] << data[10] << data[11] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        SERVO1_angle = 5;
        SERVO2_angle = 5;
        SERVO3_angle = 5;
        SERVO4_angle = 5;
        SERVO5_angle = 5;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        udp.send(data);
        std::cout << "case5_red終了" << std::endl;
    }

    // case6_red
    static void case6_red_action(UDP &udp) {
        std::cout << "<case6_redシーケンス開始>" << std::endl;
        SERVO1_angle = 6;
        SERVO2_angle = 6;
        SERVO3_angle = 6;
        SERVO4_angle = 6;
        SERVO5_angle = 6;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        udp.send(data);
        std::cout << data[7] << data[8] << data[9] << data[10] << data[11] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        SERVO1_angle = 6;
        SERVO2_angle = 6;
        SERVO3_angle = 6;
        SERVO4_angle = 6;
        SERVO5_angle = 6;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        udp.send(data);
        std::cout << "case6_red終了" << std::endl;
    }

    // case7_red
    static void case7_red_action(UDP &udp) {
        std::cout << "<case7_redシーケンス開始>" << std::endl;
        SERVO1_angle = 7;
        SERVO2_angle = 7;
        SERVO3_angle = 7;
        SERVO4_angle = 7;
        SERVO5_angle = 7;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        udp.send(data);
        std::cout << data[7] << data[8] << data[9] << data[10] << data[11] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        SERVO1_angle = 7;
        SERVO2_angle = 7;
        SERVO3_angle = 7;
        SERVO4_angle = 7;
        SERVO5_angle = 7;
        data[7] = SERVO1_angle;
        data[8] = SERVO2_angle;
        data[9] = SERVO3_angle;
        data[10] = SERVO4_angle;
        data[11] = SERVO5_angle;
        udp.send(data);
        std::cout << "case7_red終了" << std::endl;
    }

};

class PS4_Listener : public rclcpp::Node {
    public:
        PS4_Listener(const std::string &ip, int port)
            : Node("ap4_shoot"), udp_(ip, port) {
            subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy1", 10,
                std::bind(&PS4_Listener::ps4_listener_callback, this,
                          std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(),
                        "AP4_SHOOT initialized with IP: %s, Port: %d", ip.c_str(),
                        port);
        }
    
// class ArmControllerNode : public rclcpp::Node {
// public:
//     ArmControllerNode() : Node("arm_controller_node") {
//         // "/joy" トピックを購読し、コントローラー入力があるたびに joy_callback を呼ぶ
//         subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
//             "joy", 10,
//             std::bind(&ArmControllerNode::joy_callback, this, std::placeholders::_1));

//         RCLCPP_INFO(this->get_logger(), "アーム制御ノードを起動しました。PSボタンで制御を開始してください。");
//     }

private:
    // コントローラー入力があるたびに実行される関数
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // --- 1. コントローラー入力の読み取りと整形 ---
        // 各ボタンの状態を分かりやすい変数に格納
        bool SQUARE   = msg->buttons[3];
        bool CIRCLE   = msg->buttons[1];
        bool TRIANGLE = msg->buttons[2];
        bool CROSS    = msg->buttons[0];

        bool L1 = msg->buttons[4];
        bool R1 = msg->buttons[5];
        
        bool SHARE  = msg->buttons[8];
        bool OPTION = msg->buttons[9];
        bool PS     = msg->buttons[10];

        bool L3     = msg->buttons[11];
        bool R3     = msg->buttons[12];

        // 十字キー (アナログ軸として入力される)
        bool UP    = msg->axes[7] == 1.0;
        bool DOWN  = msg->axes[7] == -1.0;
        bool LEFT  = msg->axes[6] == 1.0;
        bool RIGHT = msg->axes[6] == -1.0;

        // L2/R2トリガー (-1.0 ~ 1.0 の値を 0.0 ~ 1.0 に変換)
        float L2 = (msg->axes[2] + 1.0) / 2.0;
        float R2 = (msg->axes[5] + 1.0) / 2.0;


        // デッドゾーン以下のトリガー入力を無視
        if (L2 <= TRIGGER_DEADZONE) {
            //RCLCPP_DEBUG(this->get_logger(), "L2トリガーがデッドゾーン以下: %f", L2);
            L2 = 0.0;
        }
        if (R2 <= TRIGGER_DEADZONE) {
            //RCLCPP_DEBUG(this->get_logger(), "R2トリガーがデッドゾーン以下: %f", R2);
            R2 = 0.0;
        }

        // 前回の状態を保持する static 変数
        static bool last_SHARE = false;
        //static bool last_OPTION = false;

        // SHARE のラッチ状態を保持する static 変数（初期状態は OFF とする）
        static bool SHARE_latch = false;
        //static bool OPTION_latch = false;

        if (SHARE && !last_SHARE) { // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
            SHARE_latch = !SHARE_latch;
        }
        CHANGEMODE = SHARE_latch;
        // if (OPTION && !last_OPTION) { // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
        //     OPTION_latch = !OPTION_latch;
        // }
        // last_OPTION = OPTION;
        // PtoPMODE = OPTION_latch;

        //&&&サーボ1の制御!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // 0 → 1 に変化したときだけサーボ1が10ずつが加算されていく
        if (LEFT && !prev_LEFT) {   // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
            SERVO1_angle = SERVO1_angle +5;
        }
        // 0 → 1 に変化したときだけサーボ1が10ずつ減算されていく
        else if (RIGHT && !prev_RIGHT) {   // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
            SERVO1_angle = SERVO1_angle -5;
        }
        if (SERVO1_angle > 270){
            SERVO1_angle = 270;
        }else if(SERVO1_angle < 0){
            SERVO1_angle = 0;
        }

        //&&&サーボ2の制御!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // 0 → 1 に変化したときだけサーボ2が10ずつが加算されていく
        if (UP && !prev_UP) {   // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
            SERVO2_angle = SERVO2_angle +5;
        }
        // 0 → 1 に変化したときだけサーボ4が10ずつ減算されていく
        else if (DOWN && !prev_DOWN) {   // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
            SERVO2_angle = SERVO2_angle -5;
        }
        if (SERVO2_angle > 70){
            SERVO2_angle = 70;
        }else if(SERVO2_angle < 0){
            SERVO2_angle = 0;
        }

        //&&&サーボ3の制御!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        if(L2 > 0){   //ゼロイチ判断
            L2 = 1;
        }else{
            L2 = 0;
        }
        // 0 → 1 に変化したときだけサーボ3が10ずつが加算されていく
        if (L2 && !prev_L2) {   // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
            SERVO3_angle = SERVO3_angle +5;
        }
        // 0 → 1 に変化したときだけサーボ3が10ずつ減算されていく
        else if (L1 && !prev_L1) {   // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
            SERVO3_angle = SERVO3_angle -5;
        }
        if (SERVO3_angle > 70){
            SERVO3_angle = 70;
        }else if(SERVO3_angle < 0){
            SERVO3_angle = 0;
        }


        //&&&サーボ4の制御!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // 0 → 1 に変化したときだけサーボ4が10ずつが加算されていく
        if (SQUARE && !prev_SQUARE) {   // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
            SERVO4_angle = SERVO4_angle +5;
        }
        // 0 → 1 に変化したときだけサーボ5が10ずつ減算されていく
        else if (CIRCLE && !prev_CIRCLE) {   // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
            SERVO4_angle = SERVO4_angle -5;
        }
        if (SERVO4_angle > 65){
            SERVO4_angle = 65;
        }else if(SERVO4_angle < 0){
            SERVO4_angle = 0;
        }

        //&&&サーボ5の制御!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        if(R2 > 0){   //ゼロイチ判断
            R2 = 1;
        }else{
            R2 = 0;
        }
        // 0 → 1 に変化したときだけサーボ5が10ずつが加算されていく
        if (R1 && !prev_R1) {   // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
            SERVO5_angle = SERVO5_angle +5;
        }
        // 0 → 1 に変化したときだけサーボ5が10ずつ減算されていく
        else if (R2 && !prev_R2) {   // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
            SERVO5_angle = SERVO5_angle -5;
        }
        if (SERVO5_angle > 90){
            SERVO5_angle = 90;
        }else if(SERVO5_angle < 0){
            SERVO5_angle = 0;
        }
        

        //エアシリンダの制御!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        if(PS){
            data[12] = 1;
        }else{
            data[12] = 0;
        }

        // SHAREによるモード切替
        if(CHANGEMODE == 0){            //blueモード

        // 0 → 1 に変化したときだけカウントしR_countが加算されていく
        if (L3 && !prev_L3) {   // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
            R_count++;
        }
        // 0 → 1 に変化したときだけカウントしR_countが加算されていく
        else if (R3 && !prev_R3) {   // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
            R_count = R_count + 6;
        }

        if(R_count %7 == 0 && OPTION) {       // case1_blue
            Action::case1_blue_action(udp_);
        }
        else if(R_count %7 == 1&& OPTION) {  // case2_blue
            Action::case2_blue_action(udp_);
        }
        else if(R_count %7 == 2&& OPTION) {  // case3_blue
            Action::case3_blue_action(udp_);
        }
        else if(R_count %7 == 3&& OPTION) {  // case4_blue
            Action::case4_blue_action(udp_);
        }
        else if(R_count %7 == 4&& OPTION){   // case5_blue
            Action::case5_blue_action(udp_);
        }
        else if(R_count %7 == 5 && OPTION){   // case6_blue
            Action::case6_blue_action(udp_);
        }
        else if(R_count %7 == 6&& OPTION){   // case7_blue
            Action::case7_blue_action(udp_);
        }
        }
        
        // SHAREによるモード切替
        if(CHANGEMODE == 1){            //redモード

        // 0 → 1 に変化したときだけカウントしR_countが加算されていく
        if (L3 && !prev_L3) {   // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
            R_count++;
        }
        // 0 → 1 に変化したときだけカウントしR_countが加算されていく
        else if (R3 && !prev_R3) {   // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
            R_count = R_count + 6;
        }

        if(R_count %7 == 0 && OPTION) {       // case1_red
            Action::case1_red_action(udp_);
        }
        else if(R_count %7 == 1&& OPTION) {  // case2_red
            Action::case2_red_action(udp_);
        }
        else if(R_count %7 == 2&& OPTION) {  // case3_red
            Action::case3_red_action(udp_);
        }
        else if(R_count %7 == 3&& OPTION) {  // case4_red
            Action::case4_red_action(udp_);
        }
        else if(R_count %7 == 4&& OPTION){   // case5_red
            Action::case5_red_action(udp_);
        }
        else if(R_count %7 == 5&& OPTION){   // case6_red
            Action::case6_red_action(udp_);
        }
        else if(R_count %7 == 6&& OPTION){   // case7_red
            Action::case7_red_action(udp_);
        }
        }
        

        data[7] = SERVO1_angle ;    //サーボ1に指令
        data[8] = SERVO2_angle ;    //サーボ2に指令
        data[9] = SERVO3_angle ;    //サーボ3に指令
        data[10] = SERVO4_angle ;    //サーボ4に指令
        data[11] = SERVO5_angle ;    //サーボ5に指令
        
        // 状態を更新
        prev_R1 = R1; 
        prev_R2 = R2;
        prev_R3 = R3;
        prev_L1 = L1;
        prev_L2 = L2;
        prev_L3 = L3;
        prev_LEFT = LEFT;
        prev_RIGHT = RIGHT;
        prev_UP = UP;
        prev_DOWN = DOWN;
        prev_SQUARE = SQUARE;
        prev_CIRCLE = CIRCLE;
        prev_CROSS = CROSS;
        prev_TRIANGLE = TRIANGLE;

        //std::cout << data[11] << std::endl;
        std::cout << data[7] << ", " << data[8] << ", " << data[9] <<", "<< data[10] << ", " << data[11] << ", " << data[12]<< std::endl;
        //std::cout << CHANGEMODE << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        udp_.send(data);
    }   

    // メンバ変数
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    UDP udp_;
};

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<ArmControllerNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet APOLLO4SHOOT";
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
    auto ps4_listener = std::make_shared<PS4_Listener>(IP_AP4_SHOOT, PORT_AP4_SHOOT);
    exec.add_node(ps4_listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}