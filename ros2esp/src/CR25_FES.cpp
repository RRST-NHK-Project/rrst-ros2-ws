/*
RRST-NHK-Project 2025
PS4コントローラーの入力を取得するサンプルプログラム
esp32マイコンにアクチュエータ指令を送るサンプルプログラム
*/

// 標準
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
//#include "actuator_msg/msg/actuator_msg.hpp"
#include <vector>

// 定数・変数
#define MC_PRINTF 0 // マイコン側のprintfを無効化・有効化(0 or 1)
#define front_speed 60 //前後の角度変化
#define updown_speed 50 //上下の角度変化
#define speed 40 //移動の速度
#define turn_speed 30 //回転の角度変化

#define turn_speed_v 5 //回転の角度変化
#define front_speed_v 5//前後の角度変化
#define updown_speed_v 100//上下の角度変化

#define deg 1
#define th 20
#define count 1
#define field 190

std::vector<int32_t> data(18, 0); // マイコンに送信される配列"data"


//各機構シーケンスを格納するクラス
class Action{
public:

    //回転
    static void tokei(){
        data[3] += -turn_speed;
    }
    static void hantokei(){
        data[3] += turn_speed;
    }
    
    //前後
    static void forward(){
        data[1] = front_speed;
        data[2] = -front_speed;
    }
    static void back(){
        data[1] = -front_speed;
        data[2] = front_speed;
    }
    static void up(){
        data[1] = speed;
        data[2] = speed;
    }
    static void down(){
        data[1] = -speed;
        data[2] = -speed;
    }

};

class PS4_Listener : public rclcpp::Node
{
public:
    PS4_Listener()
        : Node("ps4_listener")
    {

        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this,
                      std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("to_esp32_1", 10);

        RCLCPP_INFO(this->get_logger(),
                    "PS4 Listener initialized");
    }

private:
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {

        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        //  float LS_X = -1 * msg->axes[0];
        //  float LS_Y = msg->axes[1];
        //  float RS_X = -1 * msg->axes[3];
        //  float RS_Y = msg->axes[4];

        bool CROSS = msg->buttons[0];
        bool CIRCLE = msg->buttons[1];
        bool TRIANGLE = msg->buttons[2];
        //bool SQUARE = msg->buttons[3];

        //bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        bool L1 = msg->buttons[4];
        bool R1 = msg->buttons[5];

        // float L2_DIGITAL = (-1 * msg->axes[2] + 1) / 2;
        // float R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2;

        bool L2 = msg->buttons[6];
        bool R2 = msg->buttons[7];

        //bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        static bool last_circle = false; // 前回の状態を保持する static 変数
        static bool circle_latch = false;

        //static bool last_triangle = false;

        data[0] = MC_PRINTF; // マイコン側のprintfを無効化・有効化(0 or 1)

        if (CIRCLE && !last_circle)
        {
            circle_latch = !circle_latch;
        }

        last_circle = CIRCLE;
        data[17] = circle_latch;

        if(L2){
            data[9] -= count;
        }
        if(R2){
            data[9] += count;
        }


            data[1] = 0;
            data[2] = 0;
            data[3] = 0;
            
            
            
            if (TRIANGLE )
            {
               Action::up();
            }
            else if (CROSS)
            {
               Action::down();
            }
            else if (UP)
            {                   // 後退
                Action::forward();
            }
            else if (DOWN)
            {                   // 前進
               Action::back();
            }
            else if (L1)
            {
               Action::tokei(); // CIRCLEボタンが押されていない場合は0に設定
            }
            else if (R1)
            {
                Action::hantokei(); // CIRCLEボタンが押されていない場合は0に設定
            }
            else if (PS)
            {
                
                data[3]= field;
            }
            // last_R1 = R1;
            // last_L1 = L1;

              

        // if (PS == true){
            
        //     data[1] = 0;
        //     data[2] = 0;
        //     data[3] = 0;
        //     data[4] = 0;
        //     data[5] = 0;
        //     data[6] = 0;
        // }
        // デバッグ用（for文でcoutするとカクつく）
        std::cout << data[0] << ", " << data[1] << ", " << data[2] << ", " << data[3] << ", ";
        std::cout << data[4] << ", " << data[5] << ", " << data[6] << ", " << data[7] << ", ";
        std::cout << data[8] << ", " << data[9] << ", " << data[10] << ", " << data[11] << ", ";
        std::cout << data[12] << ", " << data[13] << ", " << data[14] << ", " << data[15] << ", ";
        std::cout << data[16] << ", " << data[17] << std::endl;

        publish_data();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    void publish_data()
    {
        auto msg = std_msgs::msg::Int32MultiArray();
        msg.data.reserve(data.size());
        for (auto &v : data)
        {
            msg.data.push_back(static_cast<int32_t>(v));
        }
        publisher_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto ps4_listener = std::make_shared<PS4_Listener>();
    exec.add_node(ps4_listener);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}