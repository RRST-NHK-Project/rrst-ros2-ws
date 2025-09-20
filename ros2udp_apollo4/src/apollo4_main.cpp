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
#define DEADZONE_L2 0.5
#define DEADZONE_R2 0.5

//R1,R2,TRIANGLEの切り替えに必要
//int R_count = 0;           // R1、R2が押された回数
int TRIANGLE_count = 0;     // TRIANGLEが押された回数
bool prev_R1 = false;       // 前回のR1の状態
bool prev_R2 = false;       // 前回のR1の状態
bool prev_TRIANGLE = false;       // 前回のTRIANGLEの状態
//bool prev_CROSS = false;        //前回のCROSSの状態

//モード切り替え
bool CHANGEMODE = false;

// 速度
int wheelspeed = 70;
int absorbspeed = 98;

//サーボ
int servo_angle = 90;        // 現在の角度（初期は90度）
int servo_step = 2;          // 1回の更新で動く角度

// サーボの組み付け時のズレを補正（度数法）
int SERVO1_CAL = 0;
int SERVO2_CAL = 0;
int SERVO3_CAL = 0;
int SERVO4_CAL = 0;
int SERVO5_CAL = 0;
int SERVO6_CAL = 0;


std::vector<int16_t> data(19, 0); // マイコンに送信される配列"data"
/*
マイコンに送信される配列"data"
debug: マイコンのprintfを有効化, MD: モータードライバー, TR: トランジスタ ,ENC: エンコーダ
| data[n] | 詳細 | 範囲 |
| ---- | ---- | ---- |
| data[0] | debug | 0 or 1 |
MD1,2,3でアームを制御(R,θ,Z)軸,MD5,6でDCを５個ずつ制御
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
        : Node("ap4_main"), udp_(ip, port) {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy0", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "AP4_MAIN initialized with IP: %s, Port: %d", ip.c_str(),
                    port);
    }

private:
    // コントローラーの入力を取得、使わない入力はコメントアウト推奨
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        //float LS_X = -1 * msg->axes[0];
        //float LS_Y = msg->axes[1];
        //float RS_X = -1 * msg->axes[3];
        // float RS_Y = msg->axes[4];

        //bool CROSS = msg->buttons[0];
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
        //float R2 = (-1 * msg->axes[5] + 1) / 2;

        //bool SHARE = msg->buttons[8];
        //bool OPTION = msg->buttons[9];
        //bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        //bool R3 = msg->buttons[12];

        //L2,R2でゼロイチ制御するためdeadzone追加
        if(L2 <= DEADZONE_L2){
            L2 = 0; 
        }

        // if(R2 <= DEADZONE_R2){
        //     R2 = 0; 
        // }

        // 前回の状態を保持する static 変数
        // static bool last_PS = false;

        // // PS のラッチ状態を保持する static 変数（初期状態は OFF とする）
        // static bool PS_latch = false;

        data[0] = MC_PRINTF; // マイコン側のprintfを無効化・有効化(0 or 1)

        // RθZの操作!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // R軸の操作
        if (UP) {       // 正転
            data[1] = 75;
        }else if (DOWN) {     // 逆転
            data[1] = -75;
        }else{
            data[1] = 0;
        }
        //θ軸の操作
        if (LEFT) {     // 正転
            data[2] = wheelspeed;
        }else if (RIGHT) {    // 逆転
            data[2] = -wheelspeed;
        }else{
            data[2] = 0;
        }
        // Z軸の操作
        if(L1) {         // 正転
            data[3] = -wheelspeed; 
        }else if(L2>0) {       // 逆転
            data[3] = wheelspeed;
        }else{
            data[3] = 0;
        }
        // サーボの手先角度操作!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // ボタン入力に応じて角度を変化
        // DS3218proなので270度までいけます
        if (SQUARE && !CIRCLE){
        // 徐々に正転
        servo_angle -= servo_step;
        if (servo_angle < 0) {// 下限で止める
            servo_angle = 0;
            }
        } else if (CIRCLE && !SQUARE){
        // 徐々に逆転
        servo_angle += servo_step;
        if (servo_angle > 270) {// 上限で止める
            servo_angle = 270;
            }    
        }
        // サーボ指令
        data[14] = servo_angle + SERVO6_CAL;

        //吸着モード切り替え!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // 0 → 1 に変化したときだけカウントしTRIANGLE_countが加算されていく
        if (TRIANGLE && !prev_TRIANGLE) {   // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
            TRIANGLE_count++;
        }
        if(TRIANGLE_count %2 == 0) {  // 全OFF
            data[5] = 0;                //上data5,6
            data[6] = 0;
            data[7] = 0;                //下data7,8
            data[8] = 0;
        }
        else if(TRIANGLE_count %2 == 1) {  // 全ON
            data[5] = absorbspeed;          //上data5,6                
            data[6] = absorbspeed;
            data[7] = absorbspeed;          //下data7,8
            data[8] = absorbspeed;
        }

        //吸着したワークを落とすサーボ操作!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        if(R1){
            data[9] = 90;
        }else{
            data[9] = 0;
        }


        // if (PS && !last_PS) { // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
        //     PS_latch = !PS_latch;
        // }
        // last_PS = PS;
        // CHANGEMODE = PS_latch;
        
        // // 展開指変更!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // //R2順方向
        // if(R2 > 0){   //ゼロイチ判断
        //     R2 = 1;
        // }else{
        //     R2 = 0;
        // }
        // // 0 → 1 に変化したときだけカウントしR_countが加算されていく
        // if (R2 && !prev_R2) {   // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
        //     R_count++;
        // }
        // //R1逆方向
        // // 0 → 1 に変化したときだけカウントしR_countが加算されていく
        // else if (R1 && !prev_R1) {   // 論理積(AND演算子よりtrue&&trueのときのみtrueとなりそれ以外はfalse)
        //     R_count = R_count + 2;
        // }
        // if(R_count %3 == 0){
        //     data[9] = 90 + SERVO1_CAL;
        //     data[10] = 90 + SERVO2_CAL;
        // }
        // else if(R_count %3 == 1){
        //     data[9] = 150 + SERVO1_CAL;
        //     data[10] = 0 + SERVO2_CAL;
        // }
        // else if(R_count %3 == 2){
        //     data[9] = 0 + SERVO1_CAL;
        //     data[10] = 150 +SERVO2_CAL;
        // }

        // if(R_count %6 == 0) {  // 垂直の状態
        //     data[9] = 90 + SERVO1_CAL;
        //     data[10] = 90 + SERVO2_CAL;
        //     data[11] = 90 + SERVO3_CAL;
        //     data[12] = 90 + SERVO4_CAL;
        //     data[13] = 100 + SERVO5_CAL;
        // }
        // else if(R_count %6 == 1) {  // 取る状態1
        //     data[9] = 0 + SERVO1_CAL;
        //     data[10] = 150 + SERVO2_CAL;
        //     data[11] = 0 + SERVO3_CAL;
        //     data[12] = 150 + SERVO4_CAL;
        //     data[13] = 150 + SERVO5_CAL;
        // }
        // else if(R_count %6 == 2) {  // 取る状態2
        //     data[9] = 150 + SERVO1_CAL;
        //     data[10] = 0 + SERVO2_CAL;
        //     data[11] = 0 + SERVO3_CAL;
        //     data[12] = 150 + SERVO4_CAL;
        //     data[13] = 150 + SERVO5_CAL;
        // }
        // else if(R_count %6 == 3) {  // 取る状態3
        //     data[9] = 150 + SERVO1_CAL;
        //     data[10] = 150 + SERVO2_CAL;
        //     data[11] = 0 + SERVO3_CAL;
        //     data[12] = 150 + SERVO4_CAL;
        //     data[13] = 150 + SERVO5_CAL;
        // }
        // else if(R_count %6 == 4){   // 取る状態4
        //     data[9] = 150 + SERVO1_CAL;
        //     data[10] = 150 + SERVO2_CAL;
        //     data[11] = 0 + SERVO3_CAL;
        //     data[12] = 0 + SERVO4_CAL;
        //     data[13] = 150 + SERVO5_CAL;
        // }
        // else if(R_count %6 == 5){   // 取る状態5
        //     data[9] = 150 + SERVO1_CAL;
        //     data[10] = 150 + SERVO2_CAL;
        //     data[11] = 0 + SERVO3_CAL;
        //     data[12] = 150 + SERVO4_CAL;
        //     data[13] = 0 + SERVO5_CAL;
        // }
        

        // if(CROSS && !prev_CROSS) {
        //     TRIANGLE_count = 1;         // 吸着割り込み(☓のときAモードにする)
        // }
        // if(TRIANGLE_count %3 == 0) {  // 全OFF(Cモード)
        //     data[5] = 0;                //上data5,6
        //     data[6] = 0;
        //     data[7] = 0;                //下data7,8
        //     data[8] = 0;
        // }
        // else if(TRIANGLE_count %3 == 1) {  // 全ON(Aモード)
        //     data[5] = absorbspeed;          //上data5,6                
        //     data[6] = absorbspeed;
        //     data[7] = absorbspeed;          //下data7,8
        //     data[8] = absorbspeed;
        // }
        // else if(TRIANGLE_count %3 == 2) {  // 上ON/下OFF(Bモード)
        //     data[5] = absorbspeed;          //上data5,6
        //     data[6] = absorbspeed;
        //     data[7] = 0;                    //下data7,8
        //     data[8] = 0;
        // }
        
        
        // PSによるモード切替
        // if(CHANGEMODE == 1){            //　受け渡しモード
        //     data[9] = 90 + SERVO1_CAL;
        //     data[10] = 90 + SERVO2_CAL;
        //     data[11] = 90 + SERVO3_CAL;
        //     data[12] = 90 + SERVO4_CAL;
        //     data[13] = 90 + SERVO5_CAL;
        // }


        // 状態を更新
        prev_R1 = R1; 
        //prev_R2 = R2;
        prev_TRIANGLE = TRIANGLE;
        //prev_CROSS = CROSS;



        // デバッグ用（for文でcoutするとカクつく）
        //std::cout << R2_count << std::endl;
        //std::cout << data[5] << ", " << data[6] << ", " << data[7] << ", " << data[8]<< std::endl; //吸着できているか確認する用
        std::cout << data[9] << ", " << data[10] << ", " << data[11] << ", " << data[12] << ", " << data[13] << std::endl;
        //std::cout << data[1] << std::endl;
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
        message.data = {data[5], data[6], data[7], data[8]};

        // RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
        publisher_->publish(message); // メッセージをpublish
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

class Params_Listener : public rclcpp::Node {
public:
    Params_Listener() : Node("ap4_main_servo_cal_listener") {
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "mr_servo_cal", 10,
            std::bind(&Params_Listener::params_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "AP4 Servo Calibrator Listener");
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
    std::string figletout = "figlet APOLLO4MAIN";
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
    auto ps4_listener = std::make_shared<PS4_Listener>(IP_AP4_MAIN, PORT_AP4_MAIN);
    auto params_listener = std::make_shared<Params_Listener>();
    auto servo_deg_publisher = std::make_shared<Servo_Deg_Publisher>();
    exec.add_node(ps4_listener);
    exec.add_node(servo_deg_publisher);
    exec.add_node(params_listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}