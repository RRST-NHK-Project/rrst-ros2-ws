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
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <vector>

#define theta_step_deg 10
#define theta_step_deg_large 30
#define z_step_deg 10
#define r_step_deg 10

bool AUTOMATIC = 0; // 0:手動モード 1:自動モード
bool LERP = 1;      // 線形補間の有効化

int m1 = 0;
int m2 = 0;
int m3 = 0;
int m4 = 0;

int target_point = 0; // 目標地点

std::vector<int32_t> data(25, 0); // マイコンに送信される配列"data"

// 自動化用に各モーターの角度を格納
std::vector<std::array<int, 4>> motor_angle_sets;

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
    // 線形補間
    inline int smooth_step(int current, int target, float alpha = 0.1f, int threshold = 5) {
        int next = static_cast<int>(current + alpha * (target - current));
        if (std::abs(target - next) <= threshold) {
            return target; // 無理やり収束させる
        }
        return next;
    }

    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        //  float LS_X = -1 * msg->axes[0];
        //  float LS_Y = msg->axes[1];
        //  float RS_X = -1 * msg->axes[3];
        //  float RS_Y = msg->axes[4];

        bool CROSS = msg->buttons[0];
        bool CIRCLE = msg->buttons[1];
        bool TRIANGLE = msg->buttons[2];
        // bool SQUARE = msg->buttons[3];

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
        static bool last_circle = false;
        static bool circle_latch = false;
        // static bool last_triangle = false;

        // data[0] = DEG_VEL; // マイコン側のprintfを無効化・有効化(0 or 1)

        if (SHARE && !last_share) {
            share_latch = !share_latch;
        }

        if (CIRCLE && !last_circle) {
            circle_latch = !circle_latch;
        }

        last_share = SHARE;
        last_circle = CIRCLE;

        AUTOMATIC = share_latch;

        if (AUTOMATIC == 0) {

            if (L1) {
                data[3] -= theta_step_deg;
            }
            if (R1) {
                data[3] += theta_step_deg;
            }

            if (L2) {
                data[3] -= theta_step_deg_large;
            }

            if (R2) {
                data[3] += theta_step_deg_large;
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

            if (PS) {
                data[1] = 0;
                data[2] = 0;
                data[3] = 0;
            }

            data[17] = circle_latch;
        }

        if (AUTOMATIC == 1) {
            ;
        }

        if (LERP) {
            // 線形補間(LERP)
            static int smooth_m1 = 0, smooth_m2 = 0, smooth_m3 = 0;

            smooth_m1 = smooth_step(smooth_m1, data[1], 0.2f);
            smooth_m2 = smooth_step(smooth_m2, data[2], 0.2f);
            smooth_m3 = smooth_step(smooth_m3, data[3], 0.2f);

            data[1] = smooth_m1;
            data[2] = smooth_m2;
            data[3] = smooth_m3;
            // 線形補間ここまで
        }

        // ハンド自動旋回
        int theta_robomas = data[3];
        int theta1_actual = theta_robomas * 15 / 142;
        int servo_deg = theta1_actual;

        data[9] = servo_deg;

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

class MotorAnglesListener : public rclcpp::Node {
public:
    MotorAnglesListener() : Node("deg_listener") {
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "motor_angles", 10,
            std::bind(&MotorAnglesListener::motor_angles_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),

                    "CR25 DEG Listener initialized");
    }

private:
    void motor_angles_listener_callback(
        const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        m1 = msg->data[0];
        m2 = msg->data[1];
        m3 = msg->data[2];
        m4 = msg->data[3];

        // std::cout << m1;
        // std::cout << m2;
        // std::cout << m3;
        // std::cout << m4 << std::endl;

        data[1] = m1;
        data[2] = m2;
        data[3] = m3;
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

class TargetPointListener : public rclcpp::Node {
public:
    TargetPointListener() : Node("target_point_listener") {
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "target_point", 10,
            std::bind(&TargetPointListener::target_point_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "CR25 Target Point Listener initialized");
    }

private:
    void target_point_listener_callback(
        const std_msgs::msg::Int32::SharedPtr msg) {

        motor_angle_sets.push_back({0, 0, 0, 0});
        motor_angle_sets.push_back({100, 200, 300, 0});
        motor_angle_sets.push_back({-100, -200, -300, 0});

        target_point = msg->data;
        // std::cout << "Target Point: " << target_point << std::endl;

        std::array<int, 4> &angles = motor_angle_sets[target_point];

        data[1] = angles[0];
        data[2] = angles[1];
        data[3] = angles[2];
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto ps4_listener = std::make_shared<PS4_Listener>();
    auto motor_angles_listener = std::make_shared<MotorAnglesListener>();
    auto target_point_listener = std::make_shared<TargetPointListener>();
    exec.add_node(motor_angles_listener);
    exec.add_node(ps4_listener);
    exec.add_node(target_point_listener);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}