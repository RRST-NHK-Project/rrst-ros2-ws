#include <chrono>
#include <cmath>
#include <thread>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

// 自作クラス
#include "include/UDP.hpp"

// スティックのデッドゾーン
#define DEADZONE_L 0.7
#define DEADZONE_R 0.3

int deg;
int truedeg;
double current_yaw = 0.0; // 現在のYAW角

// IPアドレスとポートの指定
std::string udp_ip = "192.168.8.215";
int udp_port = 5000;

std::vector<int> data = {0, 0, 0, 0, 0, 0, 0, 0, 0};

int wheelspeed = 30;
int yawspeed = 10;

// サーボの組み付け時のズレを補正（度数法）
int SERVO1_CAL = -7;
int SERVO2_CAL = 2;
int SERVO3_CAL = -7;
int SERVO4_CAL = -16;

int CORRECTION1;
int CORRECTION2;
int CORRECTION3;
int CORRECTION4;

class PS4_Listener : public rclcpp::Node {
public:
    PS4_Listener(const std::string &ip, int port)
        : Node("nhk25_mr_sd"), udp_(ip, port), initial_yaw_set(false), initial_yaw(0.0) {

        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this, std::placeholders::_1));

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", rclcpp::SensorDataQoS(),
            std::bind(&PS4_Listener::imu_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "NHK2025 MR SD initialized with IP: %s, Port: %d", ip.c_str(), port);
    }

private:
    bool initial_yaw_set; // 初回のIMUデータを取得したかどうか
    double initial_yaw;   // 起動時のYAW角
    double current_yaw;   // 現在のYAW角

    // IMUのデータを受信
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        current_yaw = yaw * 180.0 / M_PI; // [rad] -> [deg] に変換

        // 起動時のYAW角を記録
        if (!initial_yaw_set) {
            initial_yaw = current_yaw;
            initial_yaw_set = true;
            RCLCPP_INFO(this->get_logger(), "初期YAW角: %.2f [deg]", initial_yaw);
        }
    }

    // PS4コントローラーの入力を処理
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        float LS_X = -1 * msg->axes[0];
        float LS_Y = msg->axes[1];
        float RS_X = -1 * msg->axes[3];
        // float RS_Y = msg->axes[4];

        // bool CROSS = msg->buttons[0];
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

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        if (PS) {
            std::fill(data.begin(), data.end(), 0);                          // 配列をゼロで埋める
            for (int attempt = 0; attempt < 10; attempt++) {                 // 10回試行
                udp_.send(data);                                             // データ送信
                std::cout << "緊急停止！ 試行" << attempt + 1 << std::endl;  // 試行回数を表示
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 100msの遅延
            }
            rclcpp::shutdown();
        }

        float rad = atan2(LS_Y, LS_X);
        deg = rad * 180 / M_PI;
        // 135度を90度とみなしたときのズレの角度

        // XY座標での正しい角度truedeg
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

        // **目標角度を起動時のYAW角に対して相対的に補正**
        double correction = initial_yaw - current_yaw; // 初期角度を基準に補正

        if (fabs(correction) >= 5) {
            CORRECTION1 = -correction;
            CORRECTION2 = -correction;
            CORRECTION3 = correction;
            CORRECTION4 = correction;
        }

        // deadzone追加
        if ((fabs(LS_X) <= DEADZONE_R) && (fabs(LS_Y) <= DEADZONE_R) && (fabs(RS_X) <= DEADZONE_L)) {
            deg = 135;
            data[1] = 0;
            data[2] = 0;
            data[3] = 0;
            data[4] = 0;
            data[5] = deg + SERVO1_CAL + CORRECTION1;
            data[6] = deg + SERVO2_CAL + CORRECTION2;
            data[7] = deg + SERVO3_CAL + CORRECTION3;
            data[8] = deg + SERVO4_CAL + CORRECTION4;
        }

        data[1] = -wheelspeed * R2;
        data[2] = -wheelspeed * R2;
        data[3] = -wheelspeed * R2;
        data[4] = -wheelspeed * R2;
        data[5] = deg + SERVO1_CAL + CORRECTION1;
        data[6] = deg + SERVO2_CAL + CORRECTION2;
        data[7] = deg + SERVO3_CAL + CORRECTION3;
        data[8] = deg + SERVO4_CAL + CORRECTION4;

        // 独ステが扱えない範囲の変換
        if ((270 < deg) && (deg < 360)) {
            deg = deg - 180;
            data[1] = wheelspeed * R2;
            data[2] = wheelspeed * R2;
            data[3] = wheelspeed * R2;
            data[4] = wheelspeed * R2;
            data[5] = deg + SERVO1_CAL;
            data[6] = deg + SERVO2_CAL;
            data[7] = deg + SERVO3_CAL;
            data[8] = deg + SERVO4_CAL;
        }

        // 時計回りYAW回転
        if (RS_X < 0 && fabs(RS_X) >= DEADZONE_R) {
            data[5] = 180 + SERVO1_CAL;
            data[6] = 90 + SERVO2_CAL;
            data[7] = 90 + SERVO3_CAL;
            data[8] = 180 + SERVO4_CAL;
            data[1] = -yawspeed;
            data[2] = yawspeed;
            data[3] = -yawspeed;
            data[4] = yawspeed;
        }
        // 半時計回りYAW回転
        if (0 < RS_X && fabs(RS_X) >= DEADZONE_R) {
            data[5] = 180 + SERVO1_CAL;
            data[6] = 90 + SERVO2_CAL;
            data[7] = 90 + SERVO3_CAL;
            data[8] = 180 + SERVO4_CAL;
            data[1] = yawspeed;
            data[2] = -yawspeed;
            data[3] = yawspeed;
            data[4] = -yawspeed;
        }

        // std::cout << deg << std::endl;
        // デバッグ用
        std::cout << data[1] << ", " << data[2] << ", " << data[3] << ", " << data[4] << ", ";
        std::cout << data[5] << ", " << data[6] << ", " << data[7] << ", " << data[8] << ", " << std::endl;

        // std::cout << data << std::endl;
        udp_.send(data);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    UDP udp_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto ps4_listener = std::make_shared<PS4_Listener>(udp_ip, udp_port);
    rclcpp::spin(ps4_listener);
    rclcpp::shutdown();
    return 0;
}
