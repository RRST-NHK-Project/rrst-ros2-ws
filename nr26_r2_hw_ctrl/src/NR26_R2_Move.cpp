/*
RRST-NHK-Project 2026
PS4コントローラーの入力を取得するサンプルプログラム
esp32マイコンにアクチュエータ指令を送るサンプルプログラム

ボタン操作について
 ○ボタン：原点復帰
 △ボタン：押してる間アーム下がる
 ✕ボタン：押してる間アーム上がる
 上矢印：120度ずつアーム下がる
 下矢印：120度ずつアーム上がる
 □ボタン：シーケンス動作開始

 登り始めが約120度、登り終わりが300度に設定してあります

*/

// 標準
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include "std_msgs/msg/int16_multi_array.hpp"

#include <mutex>

// 以下マイコンに合わせて設定
#define TARGET_DEVICE_ID 2 // 宛先マイコンのID
#define TX16NUM 24         // 送信データ数
#define RX16NUM 17         // 受信データ数

#define MC_PRINTF 0 // マイコン側のprintfを無効化・有効化(0 or 1)

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

#define MAX_DEG 300

// constexpr size_t TX16NUM = 24;

// 定数・変数
float duty_max = 100;
float for_speed = 50;
float back_speed = 50;
float sp_yaw = 0.5;
float deadzone = 0.3; // adjust DS4 deadzone
float m1 = 120;
float m2 = 120;
float s = 200;

float deg = 3;

float v1, v2, v3, v4; // 各メカナムホイールの速度指令値
// v1:第一象限, v2:第二象限, v3:第三象限, v4:第四象限

class Action
{
public:
    static void all_up(std::vector<int16_t> &data)
    {
        data[1] += m1;
        data[2] += m2;
    }
    static void all_down(std::vector<int16_t> &data)
    {
        data[1] -= m1;
        data[2] -= m2;
    }
    static void orizin(std::vector<int16_t> &data)
    {
        data[1] = 0;
        data[2] = 0;
    }
    static void gr_up(std::vector<int16_t> &data)
    {
        data[1] -= deg;
        data[2] -= deg;
    }
    static void gr_down(std::vector<int16_t> &data)
    {
        data[1] += deg;
        data[2] += deg;
    }

    ///////ここからシーケンス動作////////
    // ここの部分をイジる
    static void climb_up(std::vector<int16_t> &data)
    {
        static int step = 0;
        static float angle = 120;
        float end_angle = 300;

        const int duration_ms = 5000;
        const int step_ms = PUBLISH_RATE_MS;
        const int steps = duration_ms / step_ms;
        const float angle_step = (300.0f - 120.0f) / steps;

        if (step == 0)
        {
            data[1] = 120;
            data[2] = 120;
            data[7] = s;
            data[8] = -s;
            data[9] = -s;
            data[10] = s;
        }

        if (step < steps)
        {
            data[7] = s;
            data[8] = -s;
            data[9] = -s;
            data[10] = s;
            angle += angle_step;
            data[1] = static_cast<int16_t>(angle);
            data[2] = static_cast<int16_t>(angle);
            step++;
        }
        else
        {
            data[1] = end_angle;
            data[2] = end_angle;
            data[0] = 0; // シーケンス終了
            step = 0;
            angle = 120;
        }
    }
};

class Omni_para : public rclcpp::Node
{
public:
    Omni_para() : Node("omni_tuner")
    {

        sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "omni_param",
            10,
            std::bind(&Omni_para::param_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "OMNI Node Started.");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
    std::vector<float> params = {0, 0, 0, 0}; // 受信したパラメータを保持

    void param_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {

        if (msg->data.size() < 4)
        {
            RCLCPP_WARN(this->get_logger(), "param message too short");
            return;
        }

        // 受信した params をコピー
        duty_max = msg->data[0];
        for_speed = msg->data[1];
        back_speed = msg->data[2];
        // v4 = msg->data[3];
    }
};

class DriverInterface : public rclcpp::Node
{
public:
    DriverInterface(uint8_t device_id)
        : Node("driver_interface_" + std::to_string(device_id)),
          device_id_(device_id)
    {

        // 配列を0で初期化
        data_.assign(TX16NUM, 0);

        // joyノードのSubscribe
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&DriverInterface::ps4_listener_callback, this, std::placeholders::_1));

        // seial_bridgeへpublish
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(device_id_), 10);

        // timer_callbackを呼び出すタイマーを作成
        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&DriverInterface::publisher_timer_callback, this));

        sensor_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(device_id_),
            10,
            std::bind(&DriverInterface::sensor_callback,
                      this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "serial_tx_%d started.", device_id_);
    }

private:
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {

        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        float LS_X = -1 * msg->axes[0];
        float LS_Y = msg->axes[1];
        float RS_X = -1 * msg->axes[3];
        float RS_Y = msg->axes[4];

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
        float R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2;

        // bool L2 = msg->buttons[6];
        // bool R2 = msg->buttons[7];

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        static bool last_up = false;
        // static bool up_latch = false;
        static bool last_down = false;
        // static bool down_latch = false;

        //  if (UP && !last_up) {
        //     up_latch = !up_latch;
        // }

        // if (CIRCLE && !last_circle) {
        //     down_latch = !down_latch;
        // }

        // last_share = SHARE;
        // last_circle = CIRCLE;

        // 以降、配列data_を操作する
        float rad = atan2(LS_Y, LS_X);

        // float X = LS_X;
        // float Y = LS_Y;
        // float W = RS_X;

        if (R2_DIGITAL >= 0.3)
        {
            float vx = cos(rad) * R2_DIGITAL;
            float vy = sin(rad) * R2_DIGITAL;

            v2 = -vy + vx; // 前右
            v4 = vy + vx;  // vy + vx; // 前左
            v1 = vy - vx;  // vy - vx; // 後左
            v3 = -vy - vx; // vy + vx; // 後右
        }
        else if (RS_X >= deadzone || R1 == 1)
        {
            v2 = sp_yaw;
            v4 = -sp_yaw;
            v1 = sp_yaw;
            v3 = -sp_yaw;
        }
        else if (RS_X <= -deadzone || L1 == 1)
        {
            v2 = -sp_yaw;
            v4 = sp_yaw;
            v1 = -sp_yaw;
            v3 = sp_yaw;
        }

        else if (
            (fabsf(LS_X) <= deadzone) && (fabsf(LS_Y) <= deadzone) && (fabsf(RS_X) <= deadzone) && (fabsf(RS_Y) <= deadzone) && (R1 == 0) && (L1 == 0))
        {
            v1 = 0.0;
            v2 = 0.0;
            v3 = 0.0;
            v4 = 0.0;
        }

        // 正規化(これで全方向安定した速度出せる)
        float max_v = std::max({fabsf(v1), fabsf(v2), fabsf(v3), fabsf(v4), 1.0f});

        v1 /= max_v;
        v2 /= max_v;
        v3 /= max_v;
        v4 /= max_v;

        if (UP != last_up && UP == true)
        {
            Action::all_up(data_);
        }
        else if (DOWN != last_down && DOWN == true)
        {
            Action::all_down(data_);
        }
        last_up = UP;
        last_down = DOWN;
        if (CIRCLE)
        {
            Action::orizin(data_);
        }

        if (CROSS)
        {
            Action::gr_up(data_);
        }
        else if (TRIANGLE)
        {
            Action::gr_down(data_);
        }

        if (SQUARE == 0)
        {
            data_[7] = static_cast<int16_t>(v1 * duty_max);
            data_[8] = static_cast<int16_t>(v2 * duty_max);
            data_[9] = static_cast<int16_t>(v3 * duty_max);
            data_[10] = static_cast<int16_t>(v4 * duty_max);
        }
        else if (SQUARE == 1)
        {
            data_[0] = 1; // シーケンス開始
        } // 配列操作ここまで

        if (data_[0] == 1)
        {
            Action::climb_up(data_);
        }
    }

    void publisher_timer_callback()
    {
        std_msgs::msg::Int16MultiArray msg;

        msg.data = data_;

        publisher_->publish(msg);
        print_data();
    }

    void print_data()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        std::cout << "TX DATA: [";
        for (size_t i = 0; i < data_.size(); ++i)
        {
            std::cout << data_[i];
            if (i + 1 < data_.size())
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }

    void
    sensor_callback(
        const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {
        // 最低限：サイズチェック
        if (msg->data.size() < RX16NUM)
        {
            RCLCPP_WARN(this->get_logger(),
                        "serial_rx_%d: data too short (%zu)",
                        device_id_, msg->data.size());
            return;
        }

        // int16_t ENC1 = msg->data[1];
        // int16_t ENC2 = msg->data[2];
        //  int16_t ENC3 = msg->data[3];
        //  int16_t ENC4 = msg->data[4];
        //  int16_t ENC5 = msg->data[5];
        //  int16_t ENC6 = msg->data[6];
        //  int16_t ENC7 = msg->data[7];
        //  int16_t ENC8 = msg->data[8];

        // int16_t SW1 = msg->data[9];
        // int16_t SW2 = msg->data[10];
        int16_t SW3 = msg->data[11];
        int16_t SW4 = msg->data[12];
        // int16_t SW5 = msg->data[13];
        // int16_t SW6 = msg->data[14];
        // int16_t SW7 = msg->data[15];
        // int16_t SW8 = msg->data[16];

        // 以降、受信データを使った処理を記述
        static uint8_t sw3_prev = 0;
        static uint8_t sw4_prev = 0;

        if (sw3_prev == 0 && SW3 == 1)
        {
            data_[5] = 1;
        }
        else if (sw4_prev == 0 && SW4 == 1)
        {
            data_[6] = 1;
        }
        else
        {
            data_[5] = 0;
            data_[6] = 0;
        }
        sw3_prev = SW3;
        sw4_prev = SW4;

        // if (ENC1 > 300)
        // {
        //     data_[1] -= 360;
        // }
        // else if (ENC2 > 300)
        // {
        //     data_[2] -= 360;
        // }

        // 受信データ処理ここまで
    }

    uint8_t device_id_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<int16_t> data_;
    std::mutex data_mutex_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;

    auto driver_interface = std::make_shared<DriverInterface>(TARGET_DEVICE_ID);
    auto omni_param = std::make_shared<Omni_para>();
    exec.add_node(driver_interface);
    exec.add_node(omni_param);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}