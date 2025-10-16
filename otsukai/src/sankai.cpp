/*
"本来なら"三回生お疲れ様会
*/

// 標準
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>
// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

// 自作クラス
#include "include/IP.hpp"
#include "include/UDP.hpp"

#define MC_PRINTF 1 // マイコン側のprintfを無効化・有効化(0 or 1)

// L2,R2のデッドゾーン
#define DEADZONE_L2 0.3
//#define DEADZONE_R2 0.3

//名前の衝突を防ぐ
static bool circle_latch = false;
static bool last_circle = false;
static bool square_latch = false;
static bool last_square = false;
static bool last_cross = false;

// 速度
int wheelspeed = 50;

//サーボ
//int servo_angle = 74;        // 現在の角度（初期は74度,真っ直ぐになるように調整）
//int servo_step1 = 2;          // 1回の更新で動く角度(l1,R1)
//int servo_step2 = 5;          // 1回の更新で動く角度(l2,R2)

std::vector<int16_t> data(22, 0); // マイコンに送信される配列"data" std~は型　int16は16bit（通信量）
/*
マイコンに送信される配列"data"
debug: マイコンのprintfを有効化,  SN: ソレノイド,　MD:　モタドラ

| data[n] | 詳細 | 範囲 |
| ---- | ---- | ---- |
| data[0] | debug | 0 or 1 |
| data[1] | MD | -100 ~ 100 |
| data[2] | MD | -100 ~ 100 |
| data[3] | MD | -100 ~ 100 |
| data[4] | MD | -100 ~ 100 |
| data[5] | MD | -100 ~ 100 |
| data[6] | MD | -100 ~ 100 |
| data[7] | servo1 | 0 ~ 270 |
| data[8] | servo2 | 0 ~ 270 |
| data[9] | servo3 | 0 ~ 270 |
| data[10] | servo4 | 0 ~ 270 |
| data[11] | servo5 | 0 ~ 270 |
| data[12] | servo6 | 0 ~ 270 |
| data[13] | servo7 | 0 ~ 270 |
| data[14] | servo8 | 0 ~ 270 |
| data[15] | SN | 0 or 1 |
| data[16] | SN | 0 or 1 |
| data[17] | SN | 0 or 1 |
| data[18] | SN | 0 or 1 |
| data[19] | SN | 0 or 1 |
| data[20] | SN | 0 or 1 |
| data[21] | SN | 0 or 1 |
| data[22] | SN | 0 or 1 |
*/

class PS4_Listener : public rclcpp::Node {
public:
    PS4_Listener(const std::string &ip, int port)
        : Node("ps4_listener"), udp_(ip, port) {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "PS4 Listener initialized");
    }

private:
    // コントローラーの入力を取得、使わない入力はコメントアウト推奨
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

        data[0] = MC_PRINTF; // マイコン側のprintfを無効化・有効化(0 or 1)

        float L2 = (-1 * msg->axes[2] + 1) / 2 ;
        float R2 = (-1 * msg->axes[5] + 1) / 2 ;
        
        //int L2OR = int(L2);
        //int R2OR = int(R2);
        //float LS_X = -1 * msg->axes[0];
        
        bool CROSS = msg->buttons[0];
        bool CIRCLE = msg->buttons[1];
        //bool TRIANGLE = msg->buttons[2];
        bool SQUARE = msg->buttons[3];
        //bool L1 = msg->buttons[4];
        //bool R1 = msg->buttons[5];
        //bool SHARE = msg->buttons[8];
        //bool OPTIONS = msg->buttons[9];
        //bool PS = msg->buttons[10];
        //bool LEFT = msg->axes[6] == 1.0;
        //bool RIGHT = msg->axes[6] == -1.0;
        //bool UP = msg->axes[7] == 1.0;
        //bool DOWN = msg->axes[7] == -1.0;

        //CIRCLEとSQUAREの動作確認用
        //if (CIRCLE) {     // 正転
        //    data[1] = wheelspeed;
        //}
        //if (SQUARE) {     // 逆転
        //    data[1] = -wheelspeed;
        //}

        //R2とL2の動作確認用
        //if (R2) {
        //    data[1] = wheelspeed + R2;
        //}
        //if (L2) {
        //    data[1] = -(wheelspeed + L2);
        //}

        //ラッチ
        if (CIRCLE && !last_circle) {
            circle_latch = !circle_latch;
            if (circle_latch) {
                square_latch = false;
            }
        }
        if (SQUARE && !last_square) {
            square_latch = !square_latch;
            if (square_latch) {
                circle_latch = false;
            }
        }
        //全キャンセル
        if (CROSS && !last_cross) {
            circle_latch = false;
            square_latch = false;
            data[1] = 0;
        }

        last_circle = CIRCLE;
        last_square = SQUARE;
        last_cross = CROSS;

        if (circle_latch) { //正転
            data[1] = wheelspeed + R2*wheelspeed;
        }
        if (square_latch) { // 逆転
            data[1] = -(wheelspeed + L2*wheelspeed);
        }

        udp_.send(data);   // データ送信

    }
   
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    UDP udp_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto ps4_listener = std::make_shared<PS4_Listener>(IP_TEST, PORT_TEST);
    exec.add_node(ps4_listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}