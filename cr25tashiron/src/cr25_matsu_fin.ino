/*
RRST-NHK-Project 2025
キャチロボ2025デバッグ用ノード
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
#include <vector>

#define DEG_VEL 1        // 0:角度/1:速度
#define front_speed 360  // 前後の角度変化
#define updown_speed 150 // 上下の角度変化
#define speed 200        // 移動の速度
#define turn_speed 20    // 回転の角度変化
#define deg 2.22

#define theta_step_deg 10
#define z_step_deg 10
#define r_step_deg 10

bool MANUALMODE = true;
bool hand = false; // 手動モードでのハンド開閉

std::vector<int16_t> data(25, 0); // マイコンに送信される配列"data"

class PS4_Listener : public rclcpp::Node {
public:
    PS4_Listener()
        : Node("ps4_listener") {

        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this,
                      std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("to_esp32_0", 10);

        RCLCPP_INFO(this->get_logger(),
                    "PS4 Listener initialized");
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
        
        static bool last_circle = false; // 前回の状態を保持する static 変数
        static bool circle_latch = false;
        
        
        // static bool last_triangle = false;

        data[0] = DEG_VEL; // マイコン側のprintfを無効化・有効化(0 or 1)

        if (SHARE && !last_share) {
            share_latch = !share_latch;
        }

        last_share = SHARE;
        MANUALMODE = share_latch;

        if (CIRCLE && !last_circle) {
            circle_latch = !circle_latch;
        }

        last_circle = CIRCLE;
        hand = circle_latch;
    
        data[17] =0; // ハンドの初期化

        // L1押下で増加、R1押下で減少
        if (L1) {
            data[3] -= theta_step_deg;
        }
        if (R1) {
            data[3] += theta_step_deg;
        }

        if (UP) {
            data[1] += z_step_deg;
            data[2] += z_step_deg;
        }
        if (DOWN) {
            data[1] -= z_step_deg;
            data[2] -= z_step_deg;
        }

        if (TRIANGLE) {
            data[1] -= r_step_deg;
            data[2] += r_step_deg;
        }
        if (CROSS) {
            data[1] += r_step_deg;
            data[2] -= r_step_deg;
        }
        if (hand == true) {
            data[17] = 1;
        }


        int theta_robomas = data[3];
        int theta1_actual = theta_robomas * 15 / 142;
        int servo_deg = theta1_actual;

        data[9] = servo_deg; // サーボ1
        // if (R1) {
        //     data[3] = -10; // 押している間だけ上方向
        // } else if (L1) {
        //     data[3] = 10; // 押している間だけ下方向
        // } else {
        //     data[3] = 0; // どちらも押していなければ停止
        // }

        publish_data();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    void publish_data() {
        auto msg = std_msgs::msg::Int32MultiArray();
        msg.data.reserve(data.size());
        for (auto &v : data) {
            msg.data.push_back(static_cast<int32_t>(v));
        }
        publisher_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto ps4_listener = std::make_shared<PS4_Listener>();
    exec.add_node(ps4_listener);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}