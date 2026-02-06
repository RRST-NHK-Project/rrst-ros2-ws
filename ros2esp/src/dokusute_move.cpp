#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include <mutex>
#include <std_msgs/msg/float32_multi_array.hpp>
// #include "actuator_msg/msg/actuator_msg.hpp"
#include <vector>

// 定数・変数
#define MC_PRINTF 0 // マイコン側のprintfを無効化・有効化(0 or 1)

constexpr size_t TX16NUM = 24;

// スティックのデッドゾーン
#define DEADZONE_L 0.3
#define DEADZONE_R 0.3

// 定数k
#define k 0.05

// PIDパラメータ（チューニングが必要）
#define speed_Kp 0.3  // 速度Pゲイン
#define speed_Ki 0.05 // 速度Iゲイン
#define speed_Kd 0.05 // 速度Dゲイン

#define speed_limit 30
#define deg_limit 360
#define DPAD_SPEED 30

bool AGRESSIVEMODE = false; // 暴走モードの初期値0
bool LATERALMOTIONMODE = false;

// グローバル変数（角度一覧）
int deg = 0;
int previous_deg = 0;
int truedeg = 0;
int desired_deg = 0;
int measured_deg = 0;

// PID用の内部変数
double speed_Error = 0.0;
double speed_last_Error = 0.0;
double speed_Integral = 0.0;
double speed_Differential = 0.0;
double speed_Output = 0.0;

// 制御周期 [秒]
const double dt = 0.05; // 50ms

// 速度
int wheelspeed = 128;
int yawspeed = 10;
int previous_speed = 0;
int desired_speed = 30;
int measured_speed = 0;
int LATERALMOTION_speed = 15;
// static double current_motor_command = 0.0;

// サーボの組み付け時のズレを補正（度数法）
int SERVO1_CAL = 0;
int SERVO2_CAL = 0;
int SERVO3_CAL = 0;
int SERVO4_CAL = 0;

// 最近傍点距離の格納
float min_distance = 0;
bool front_cleared = false;

class dokusute_para : public rclcpp::Node {
public:
    dokusute_para() : Node("dokusute_param_receiver") {

        sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "dokusute_param",
            10,
            std::bind(&dokusute_para::param_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "DOKUSUTE Node Started.");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
    std::vector<float> params = {0, 0, 0, 0}; // 受信したパラメータを保持

    void param_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

        if (msg->data.size() < 4) {
            RCLCPP_WARN(this->get_logger(), "param message too short");
            return;
        }

        // 受信した params をコピー
        wheelspeed = msg->data[0];
        yawspeed = msg->data[1];
        desired_speed = msg->data[2];
        LATERALMOTION_speed = msg->data[3];

        //  data[0] = params[0]; // v1
        //  data[1] = params[1]; // v2
        //  data[2] = params[2]; // v3
        //  data[3] = params[3]; // v4

        // // ここでモータ制御に送ったり何でもできる
        // std::cout << "Received params:" << std::endl;
        // std::cout << " v1=" << params[0]
        //           << " v2=" << params[1]
        //           << " v3=" << params[2]
        //           << " v4=" << params[3]
        //           << std::endl;
    }
};

class PS4_Listener : public rclcpp::Node {
public:
    PS4_Listener(uint8_t device_id)
        : Node("ps4_listener"),
          device_id_(device_id) {
        data.assign(TX16NUM, 0);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy",
            rclcpp::SensorDataQoS(),
            std::bind(&PS4_Listener::ps4_listener_callback, this, std::placeholders::_1));

        param_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "dokusute_param",
            10,
            std::bind(&PS4_Listener::param_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(device_id_), 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50), // 20Hz
            std::bind(&PS4_Listener::timer_callback, this));

        RCLCPP_INFO(get_logger(),
                    "PS4 → serial_tx_%d started (timer publish)", device_id_);
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr param_sub_;

    void param_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 4)
            return;

        wheelspeed = msg->data[0];
        yawspeed = msg->data[1];
        desired_speed = msg->data[2];
        LATERALMOTION_speed = msg->data[3];

        // RCLCPP_INFO(this->get_logger(),
        //     "Param updated: wheel=%d yaw=%d desired=%d lateral=%d",
        //     wheelspeed, yawspeed, desired_speed, LATERALMOTION_speed);
    }

    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

        // コントローラーの入力を取得、使わない入力はコメントアウト推奨

        // ここから先は data を mutex で守りながら更新する
        std::lock_guard<std::mutex> lock(datamutex_);

        data[0] = MC_PRINTF; // マイコン側のprintfを無効化・有効化(0 or 1)

        float LS_X = -1 * msg->axes[0];
        float LS_Y = msg->axes[1];
        float RS_X = -1 * msg->axes[3];
        // float RS_Y = msg->axes[4];

        // bool CROSS = msg->buttons[0];
        //  bool CIRCLE = msg->buttons[1];
        //  bool TRIANGLE = msg->buttons[2];
        //  bool SQUARE = msg->buttons[3];

        bool LEFT = msg->axes[6] == 1.0;
        bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        bool L1 = msg->buttons[4];
        // bool R1 = msg->buttons[5];

        // float L2 = (-1 * msg->axes[2] + 1) / 2;
        float R2 = (-1 * msg->axes[5] + 1) / 2;

        // bool SHARE = msg->buttons[8];
        bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        bool R3 = msg->buttons[12];

        static bool last_option = false; // 前回の状態を保持する static 変数
        // OPTION のラッチ状態を保持する static 変数（初期状態は OFF とする）
        static bool option_latch = false;

        static bool last_R3 = false;
        static bool R3_latch = false;

        if (OPTION && !last_option) {
            option_latch = !option_latch;
        }
        if (R3 && !last_R3) {
            R3_latch = !R3_latch;
        }

        last_option = OPTION;
        AGRESSIVEMODE = option_latch;
        last_R3 = R3;
        LATERALMOTIONMODE = R3_latch;

        // if (AGRESSIVEMODE == 0) {
        //     wheelspeed = 128;
        //     data[11] = 1; // LED光らない
        // }
        // if (AGRESSIVEMODE == 1) {
        //     wheelspeed = 50;
        //     data[11] = 0; // LED光る
        // }

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        // PSボタンで緊急停止 TODO:復帰機能の実装
        // if (PS && !ps_prev) {
        //     ps_state = !ps_state; // トグル切り替え
        //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // }
        // ps_prev = PS;       // 現在のPSボタンの状態を保存
        // soft_es = ps_state; // 緊急停止状態を保存

        // if (soft_es) {
        //     data[0] = -1;
        //     std::cout << "緊急停止！" << std::endl;
        // } else if (MC_PRINTF == 0 || MC_PRINTF == 1) {
        //     data[0] = MC_PRINTF;
        // } else {
        //     data[0] = 1;
        // }

        // if (L1) {
        //     Automation::automatic_cruise(udp_);
        //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // }

        float rad = atan2(LS_Y, LS_X);
        deg = rad * 180 / M_PI;
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

        // deadzone追加
        if ((fabs(LS_X) <= DEADZONE_R) && (fabs(LS_Y) <= DEADZONE_R) && (fabs(RS_X) <= DEADZONE_L)) {
            deg = 135;
            data[1] = 0;
            data[2] = 0;
            data[3] = 0;
            data[4] = 0;
            data[9] = deg + SERVO1_CAL;
            data[10] = deg + SERVO2_CAL;
            data[11] = deg + SERVO3_CAL;
            data[12] = deg + SERVO4_CAL;
        }

        data[1] = -wheelspeed * R2;
        data[2] = -wheelspeed * R2;
        data[3] = -wheelspeed * R2;
        data[4] = -wheelspeed * R2;
        data[9] = deg + SERVO1_CAL;
        data[10] = deg + SERVO2_CAL;
        data[11] = deg + SERVO3_CAL;
        data[12] = deg + SERVO4_CAL;

        if (LEFT) {
            deg = 45;
            data[1] = -wheelspeed * R2;
            data[2] = -wheelspeed * R2;
            data[3] = -wheelspeed * R2;
            data[4] = -wheelspeed * R2;
        }
        if (RIGHT) {
            deg = 45;
            data[1] = wheelspeed * R2;
            data[2] = wheelspeed * R2;
            data[3] = wheelspeed * R2;
            data[4] = wheelspeed * R2;
        }
        if (UP) {
            deg = 135;
            data[1] = -wheelspeed * R2;
            data[2] = -wheelspeed * R2;
            data[3] = -wheelspeed * R2;
            data[4] = -wheelspeed * R2;
        }
        if (DOWN) {
            deg = 135;
            data[1] = wheelspeed * R2;
            data[2] = wheelspeed * R2;
            data[3] = wheelspeed * R2;
            data[4] = wheelspeed * R2;
        }

        // 独ステが扱えない範囲の変換
        if ((270 < deg) && (deg < 360)) {
            deg = deg - 180;
            data[1] = wheelspeed * R2;
            data[2] = wheelspeed * R2;
            data[3] = wheelspeed * R2;
            data[4] = wheelspeed * R2;
            data[9] = deg + SERVO1_CAL;
            data[10] = deg + SERVO2_CAL;
            data[11] = deg + SERVO3_CAL;
            data[12] = deg + SERVO4_CAL;
        }

        // 角度だけ横移動
        if (R3_latch == 0) {
            data[9] = deg + SERVO1_CAL;
            data[10] = deg + SERVO2_CAL;
            data[11] = deg + SERVO3_CAL;
            data[12] = deg + SERVO4_CAL;
        }
        if (R3_latch == 1) {
            data[9] = 45 + SERVO1_CAL;
            data[10] = 45 + SERVO2_CAL;
            data[11] = 45 + SERVO3_CAL;
            data[12] = 45 + SERVO4_CAL;
            if (LEFT) {
                data[1] = -LATERALMOTION_speed;
                data[2] = -LATERALMOTION_speed;
                data[3] = -LATERALMOTION_speed;
                data[4] = -LATERALMOTION_speed;
            }
            if (RIGHT) {
                data[1] = LATERALMOTION_speed;
                data[2] = LATERALMOTION_speed;
                data[3] = LATERALMOTION_speed;
                data[4] = LATERALMOTION_speed;
            }
        }

        // 時計回りYAW回転
        if (RS_X > 0 && fabs(RS_X) >= DEADZONE_R) {
            data[9] = 180 + SERVO1_CAL;
            data[10] = 90 + SERVO2_CAL;
            data[11] = 90 + SERVO3_CAL;
            data[12] = 180 + SERVO4_CAL;
            data[1] = -yawspeed;
            data[2] = yawspeed;
            data[3] = -yawspeed;
            data[4] = yawspeed;
        }
        // 半時計回りYAW回転
        if (0 > RS_X && fabs(RS_X) >= DEADZONE_R) {
            data[9] = 180 + SERVO1_CAL;
            data[10] = 90 + SERVO2_CAL;
            data[11] = 90 + SERVO3_CAL;
            data[12] = 180 + SERVO4_CAL;
            data[1] = yawspeed;
            data[2] = -yawspeed;
            data[3] = yawspeed;
            data[4] = -yawspeed;
        }

        //    デバッグ用（for文でcoutするとカクつく）
        std::cout << data[0] << ", " << data[1] << ", " << data[2] << ", " << data[3] << ", ";
        std::cout << data[4] << ", " << data[5] << ", " << data[6] << ", " << data[7] << ", ";
        std::cout << data[8] << ", " << data[9] << ", " << data[10] << ", " << data[11] << ", ";
        std::cout << data[12] << ", " << data[13] << ", " << data[14] << ", " << data[15] << ", ";
        std::cout << data[16] << ", " << data[17] << std::endl;
    }

    void timer_callback() {
        std_msgs::msg::Int16MultiArray msg;
        msg.data = data;
        publisher_->publish(msg);
    }

    void print_data() {
        //     std::lock_guard<std::mutex> lock(data_mutex_);
        //     std::cout << "TX DATA: [";
        //     for (size_t i = 0; i < data_.size(); ++i)
        //     {
        //         std::cout << data_[i];
        //         if (i + 1 < data_.size())
        //             std::cout << ", ";
        //     }
        //     std::cout << "]" << std::endl;
    }
    uint8_t device_id_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<int16_t> data;
    std::mutex datamutex_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;

    uint8_t device_id = 2; // serial_bridge_node と一致
    auto ps4_listener = std::make_shared<PS4_Listener>(device_id);
    exec.add_node(ps4_listener);
    exec.add_node(std::make_shared<dokusute_para>());
    exec.spin();

    rclcpp::shutdown();
    return 0;
}