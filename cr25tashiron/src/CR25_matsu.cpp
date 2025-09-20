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
#include "actuator_msg/msg/actuator_msg.hpp"
#include <vector>

// 計算
//#include <cmath>


/*メッセージの定義
# 共通
int32 actuator_id          # アクチュエータの種別ごと（例：MD1とSERVO1は共存可能）
int32 actuator_type        # アクチュエータ種別 (0:デバッグ, 1:モタドラ, 2:サーボ, 3:ソレノイド, 4:ロボマス)
string actuator_type_name  # "MD", "SERVO", "SV", "ROBOMAS"
bool enable                # 有効・無効

# --- モタドラ用 ---
int32 motor_duty           # [%]
int32 motor_target_rpm     # [RPM]
int32 motor_target_pos     #
int32 motor_target_torque  #

# --- サーボ用 ---
int32 servo_angle_degree   # [deg]
int32 servo_speed          #

# --- ソレノイドバルブ用 ---
bool solenoid_state        # [True/False]

# --- ロボマス用 ---
int32 robomas_target_angle # [degree]
*/
#define MC_PRINTF 0 // マイコン側のprintfを無効化・有効化(0 or 1)
#define front_speed 720 //前後の角度変化
#define updown_speed 360 //上下の角度変化
#define speed 200 //移動の速度
#define turn_speed 30 //回転の角度変化

#define turn_speed_v 5 //回転の角度変化
#define front_speed_v 200//前後の角度変化
#define updown_speed_v 200//上下の角度変化
#define deg 1
#define th 20
#define count 1
#define field 190
bool MANUALMODE = false;

std::vector<int16_t> data(25, 0); // マイコンに送信される配列"data"


//各機構シーケンスを格納するクラス
class Action{
public:
    static void tokei(){
        data[3] -= turn_speed*deg;
    }
    static void hantokei(){
        data[3] += turn_speed*deg;
    }
    static void tokei_v(){
        data[3] -= turn_speed_v;
    }
    static void hantokei_v(){
        data[3] += turn_speed_v;
    }
    static void first_position(){
        data[3] = -90*deg;
    }
    static void second_position(){
        data[3] = 90*deg;
    }
    
    static void shoot(){
        data[3] = -90;
    }


    static void forward(){
        data[1] = front_speed;
        data[2] = -front_speed;
    }
    static void back(){
        data[1] = -front_speed;
        data[2] = front_speed;
    }
    static void forward_v(){
        data[1] = front_speed;
        data[2] = -front_speed;
    }
    static void back_v(){
        data[1] = -front_speed;
        data[2] = front_speed;
    
    }
    static void up(){
        data[1] = updown_speed;
        data[2] = updown_speed;
    }
    static void down(){
        data[1] = -updown_speed;
        data[2] = -updown_speed;
    }

     static void up_v(){
        data[1] = updown_speed_v;
        data[2] = updown_speed_v;
    }
    static void down_v(){
        data[1] = -updown_speed_v;
        data[2] = -updown_speed_v;
    }

    static void sand(){
        data[17] = 1;
    }
    static void drop(){
        data[17] = 0;
    }
    static void left(){
        data[6] = -th;
    }
    static void right(){
        data[6] = th;
    }
    static void forward_m(){
        data[4] = speed;
        data[5] = -speed;
    }
    static void back_m(){
        data[4] = -speed;
        data[5] = speed;
    }
    static void up_m(){
        data[4] = speed;
        data[5] = speed;
    }
    static void down_m(){
        data[4] = -speed;
        data[5] = -speed;
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

        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("to_esp32_0", 10);

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
        bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        bool L1 = msg->buttons[4];
        bool R1 = msg->buttons[5];

        // float L2_DIGITAL = (-1 * msg->axes[2] + 1) / 2;
        // float R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2;

        bool L2 = msg->buttons[6];
        bool R2 = msg->buttons[7];

        bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        static bool last_share = false; // 前回の状態を保持する static 変数
        static bool share_latch = false;

        static bool last_R1 =false;
         static bool last_L1 =false;
        //static bool last_triangle = false;

        data[0] = MC_PRINTF; // マイコン側のprintfを無効化・有効化(0 or 1)

        if (SHARE && !last_share)
        {
            share_latch = !share_latch;
        }

        last_share = SHARE;
        MANUALMODE = share_latch;

        if(L2){
            data[9] -= count;
        }
        if(R2){
            data[9] += count;
        }



        //自動モード
        if (MANUALMODE == false)
        {
            //std::cout << "自動モード" << std::endl;

            data[1] = 0;
            data[2] = 0;
            //data[3] = 0;
             data[7] = 0;
            data[8]= 0; 
            // if(L2)
            // {
            // Action::first_position();
            // data[8]= 1;
            // }
            // if(R2)
            // {
            // Action::second_position();
            // data[8]= 1;
            // }
            if (CIRCLE )
            {
               Action::sand();
            }
            if (TRIANGLE )
            {
               Action::up();
               data[8]= 1;
            }
            else if (CROSS)
            {
               Action::down();
               data[8]= 1;
            }
            else if (SQUARE)
            {
               Action::drop();
            }
            else if (UP)
            {                   // 後退
                Action::forward();
                data[8]= 1;
            }
            else if (DOWN)
            {                   // 前進
               Action::back();
               data[8]= 1;
            }
            else if (L1  && !last_L1 )
            {
               Action::tokei(); // CIRCLEボタンが押されていない場合は0に設定
               data[8]= 1;
            }
            else if (R1  && !last_R1)
            {
                Action::hantokei(); // CIRCLEボタンが押されていない場合は0に設定
                data[8]= 1;
            }
            else if (PS)
            {
                
                data[3]= field;
            }
            last_R1 = R1;
            last_L1 = L1;

        }
        //手動モード
        else if (MANUALMODE == true)
        {
            //std::cout << "手動モード" << std::endl;
        
            data[1] = 0;
            data[2] = 0;
            
            data[7] = 1;
           // data[4] = 0;
           // data[5] = 0;
            //data[6] = 0;
             //永井清流逆運動学
            //arctan2(Y, X)でラジアン、atan2(Y, X)*180/M_PIで度
            //atan2(LS_X, LS_Y)*180/M_PI = data[3]
             if (CIRCLE )
            {
               Action::sand();
            }
            if (TRIANGLE )
            {
               Action::up_v();
               data[8]= 1;
            }
            else if (CROSS)
            {
               Action::down_v();
               data[8]= 1;
            }
            else if (SQUARE)
            {
               Action::drop();
            }
            else if (UP)
            {                   // 後退
                Action::forward_v();
                data[8]= 1;
            }
            else if (DOWN)
            {                   // 前進
               Action::back_v();
               data[8]= 1;
            }
            else if (L1  && !last_L1 )
            {
               Action::tokei_v(); // CIRCLEボタンが押されていない場合は0に設定
               data[8]= 1;
            }
            else if (R1  && !last_R1)
            {
                Action::hantokei_v(); // CIRCLEボタンが押されていない場合は0に設定
                data[8]= 1;
            }
            else if (PS)
            {
                
                data[3]= field;
            }
            last_R1 = R1;
            last_L1 = L1;

            
            // if (CIRCLE)
            // {
            //    Action::sand();
            // }
            // if (TRIANGLE)
            // {
            //    Action::up_m();
            // }
            // else if (CROSS)
            // {
            //    Action::down_m();
            // }
            // else if (SQUARE)
            // {
            //    Action::drop();
            // }
            // else if (UP)
            // {                   // 後退
            //     Action::forward_m();
            // }
            // else if (DOWN)
            // {                   // 前進
            //    Action::back_m();
            // }
            // else if (L1)
            // {
            //    Action::left(); // CIRCLEボタンが押されていない場合は0に設定
            // }
            // else if (R1)
            // {
            //     Action::right(); // CIRCLEボタンが押されていない場合は0に設定
            // }
            // // data[4] = 0;
            // data[5] = 0;
            // data[6] = 0;
        }

       

        // if (PS == true){
            
        //     data[1] = 0;
        //     data[2] = 0;
        //     data[3] = 0;
        //     data[4] = 0;
        //     data[5] = 0;
        //     data[6] = 0;
        // }
        // デバッグ用（for文でcoutするとカクつく）
        // std::cout << data[0] << ", " << data[1] << ", " << data[2] << ", " << data[3] << ", ";
        // std::cout << data[4] << ", " << data[5] << ", " << data[6] << ", " << data[7] << ", ";
        // std::cout << data[8] << ", " << data[9] << ", " << data[10] << ", " << data[11] << ", ";
        // std::cout << data[12] << ", " << data[13] << ", " << data[14] << ", " << data[15] << ", ";
        // std::cout << data[16] << ", " << data[17] << ", " << data[18] << std::endl;
        //std::cout << "MANUALMODE = " << data[7] << std::endl;
        //std::cout << "MANUALMODE = " << data[8] << std::endl;
        std::cout << " MANUALMODE =" << data[7] << std::endl;

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