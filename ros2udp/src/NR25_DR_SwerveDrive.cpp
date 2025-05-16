/*
RRST-NHK-Project 2025
ダンク機独ステ
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
#include "std_msgs/msg/int32_multi_array.hpp"

// 自作クラス
#include "include/IP.hpp"
#include "include/UDP.hpp"

#define MC_PRINTF 0 // マイコン側のprintfを無効化・有効化(0 or 1)

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
#define DPAD_SPEED 30 // 方向パッド入力時の目標速度
bool AGRESSIVEMODE = false;//暴走モードの初期値0

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
int wheelspeed = 50;
int yawspeed = 10;
int previous_speed = 0;
int desired_speed = 30;
int measured_speed = 0;
//static double current_motor_command = 0.0;

// サーボの組み付け時のズレを補正（度数法）
int SERVO1_CAL = 15;
int SERVO2_CAL = -6;
int SERVO3_CAL = 10;
int SERVO4_CAL = 20;

// 最近傍点距離の格納
float min_distance = 0;
bool front_cleared = false;

std::vector<int16_t> data(19, 0); // マイコンに送信される配列"data"
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
| data[11] | TR1 | 0 or 1|
| data[12] | TR2 | 0 or 1|
| data[13] | TR3 | 0 or 1|
| data[14] | TR4 | 0 or 1|
| data[15] | TR5 | 0 or 1|
| data[16] | TR6 | 0 or 1|
| data[17] | TR7 | 0 or 1|
| data[18] | TR8 | 0 or 1|
*/

// 自動化クラス（実装途中）
class Automation {
public:
    // 高速自動走行（自動加速、障害物を検知したら停止）
    static void automatic_cruise(UDP &udp) {
        const int steps = 20;          // 加減速のステップ数
        const double maxOutput = 20.0; // 最大出力
        // const int cruiseTimeMs = 2000; // 巡航時間（ミリ秒）
        const int intervalMs = 50; // ステップごとの待機時間
        bool skip_while = false;

        std::cout << "<自動加速開始>" << std::endl;
        // 加速フェーズ（0 → maxOutput）
        for (int i = 0; i <= steps; ++i) {
            // 前方の障害物及び緊急停止の検知
            if (front_cleared == false || data[0] == -1) {
                std::cout << "<障害物検知>" << std::endl;
                data[1] = 0;
                data[2] = 0;
                data[3] = 0;
                data[4] = 0;
                udp.send(data);
                std::cout << "<停止！>" << std::endl;
                skip_while = true;
                break;
            }
            double t = static_cast<double>(i) / steps; // 0〜1
            double output = maxOutput * t * t;         // 2次関数加速
            data[1] = -output;
            data[2] = -output;
            data[3] = -output;
            data[4] = -output;
            std::cout << output << std::endl;
            udp.send(data);
            std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
        }

        if (skip_while == false) {
            while (1) {
                // 巡航フェーズ（maxOutput）
                // 前方の障害物及び緊急停止の検知
                if (front_cleared == false || data[0] == -1) {
                    std::cout << "障害物検知" << std::endl;
                    data[1] = 0;
                    data[2] = 0;
                    data[3] = 0;
                    data[4] = 0;
                    udp.send(data);
                    std::cout << "<停止！>" << std::endl;
                    break;
                }
                data[1] = -maxOutput;
                data[2] = -maxOutput;
                data[3] = -maxOutput;
                data[4] = -maxOutput;
                udp.send(data);
                std::cout << "<巡航中>" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
            }
        }

        skip_while = false;

        // 減速フェーズ（maxOutput → 0） 減速フェーズいらないかも？？？
        // for (int i = 0; i <= steps; ++i) {
        //     if (front_cleared == false) {
        //         std::cout << "障害物検知" << std::endl;
        //         data[1] = 0;
        //         data[2] = 0;
        //         data[3] = 0;
        //         data[4] = 0;
        //         udp.send(data);
        //         std::cout << "<停止！>" << std::endl;
        //         break;
        //     }
        //     double t = static_cast<double>(i) / steps;     // 0〜1
        //     double output = maxOutput * (1 - t) * (1 - t); // 2次関数減速
        //     data[1] = output;
        //     data[2] = output;
        //     data[3] = output;
        //     data[4] = output;
        //     std::cout << output << std::endl;
        //     udp.send(data);
        //     std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
        // }
    }
};

class PS4_Listener : public rclcpp::Node {
public:
    PS4_Listener(const std::string &ip, int port)
        : Node("nhk25_dr_sd"), udp_(ip, port) {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy1", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "NHK2025 DR SD initialized with IP: %s, Port: %d", ip.c_str(),
                    port);
    }

private:
    // コントローラーの入力を取得、使わない入力はコメントアウト推奨
    double PID(int measured_speed) {
        double error = desired_speed - measured_speed;
        double derivative = (error - speed_last_Error) / dt;
        // 積分項の一時更新
        double tentative_integral = speed_Integral + (error + speed_last_Error) * dt / 2.0;
        double output = speed_Kp * error + speed_Ki * tentative_integral + speed_Kd * derivative;

        // 変更点：出力が上限・下限を超えた場合、積分更新を抑制（アンチウィンドアップ処理）
        if (output > speed_limit) {
            output = speed_limit;
            if (error > 0) { // 正のエラーの場合、積分更新しない
                tentative_integral = speed_Integral;
            }
        } else if (output < -speed_limit) {
            output = -speed_limit;
            if (error < 0) {
                tentative_integral = speed_Integral;
            }
        }
        speed_Integral = tentative_integral;
        speed_last_Error = error;
        return output;
        // PID計算（台形則による積分計算をそのまま使用）
        // speed_Error = desired_speed - measured_speed;

        // speed_Integral += (speed_Error + speed_last_Error) * dt/ 2.0;
        // speed_Differential = (speed_Error - speed_last_Error) / dt;
        // // 各サンプリングごとにPID出力を再計算
        // speed_Output = (speed_Kp * speed_Error) + (speed_Ki * speed_Integral) + (speed_Kd * speed_Differential);
        // speed_last_Error = speed_Error;
        // return speed_Output;
    }

    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
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
        //bool R1 = msg->buttons[5];

        // float L2 = (-1 * msg->axes[2] + 1) / 2;
        float R2 = (-1 * msg->axes[5] + 1) / 2;

        // bool SHARE = msg->buttons[8];
        bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];
        static bool last_option = false; // 前回の状態を保持する static 変数
        // OPTION のラッチ状態を保持する static 変数（初期状態は OFF とする）
        static bool option_latch = false;

    if (OPTION && !last_option) {
            option_latch = !option_latch;
        }

        last_option = OPTION;
        AGRESSIVEMODE = option_latch;

        if(AGRESSIVEMODE == 0){
            wheelspeed = 50;
            data[11] = 0;//LED光らない
        }
        if(AGRESSIVEMODE == 1){
            wheelspeed = 75;
            data[11] = 1;//LED光る

        }

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

        if (L1) {
            Automation::automatic_cruise(udp_);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

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
                data[7] = deg + SERVO1_CAL;
                data[8] = deg + SERVO2_CAL;
                data[9] = deg + SERVO3_CAL;
                data[10] = deg + SERVO4_CAL;
            }

            data[1] = -wheelspeed * R2;
            data[2] = -wheelspeed * R2;
            data[3] = -wheelspeed * R2;
            data[4] = -wheelspeed * R2;
            data[7] = deg + SERVO1_CAL;
            data[8] = deg + SERVO2_CAL;
            data[9] = deg + SERVO3_CAL;
            data[10] = deg + SERVO4_CAL;

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
                data[7] = deg + SERVO1_CAL;
                data[8] = deg + SERVO2_CAL;
                data[9] = deg + SERVO3_CAL;
                data[10] = deg + SERVO4_CAL;
            }

            // 時計回りYAW回転
            if (RS_X > 0 && fabs(RS_X) >= DEADZONE_R) {
                data[7] = 180 + SERVO1_CAL;
                data[8] = 90 + SERVO2_CAL;
                data[9] = 90 + SERVO3_CAL;
                data[10] = 180 + SERVO4_CAL;
                data[1] = -yawspeed;
                data[2] = yawspeed;
                data[3] = -yawspeed;
                data[4] = yawspeed;
            }
            // 半時計回りYAW回転
            if (0 > RS_X && fabs(RS_X) >= DEADZONE_R) {
                data[7] = 180 + SERVO1_CAL;
                data[8] = 90 + SERVO2_CAL;
                data[9] = 90 + SERVO3_CAL;
                data[10] = 180 + SERVO4_CAL;
                data[1] = yawspeed;
                data[2] = -yawspeed;
                data[3] = yawspeed;
                data[4] = -yawspeed;
            }
        

        // デバッグ用 *NOTE:for文でcoutするとカクつくからこの記述
        // std::cout << data[0] << ", " << data[1] << ", " << data[2] << ", " << data[3] << ", ";
        // std::cout << data[4] << ", " << data[5] << ", " << data[6] << ", " << data[7] << ", ";
        // std::cout << data[8] << ", " << data[9] << ", " << data[10] << ", " << data[11] << ", ";
        // std::cout << data[12] << ", " << data[13] << ", " << data[14] << ", " << data[15] << ", ";
        // std::cout << data[16] << ", " << data[17] << ", " << data[18] << std::endl;
        //std::cout << data[1] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // 現在の状態を次回のために保存
        last_option = OPTION;
        AGRESSIVEMODE = option_latch;

        udp_.send(data);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    UDP udp_;
};

class Servo_Deg_Publisher : public rclcpp::Node {
public:
    Servo_Deg_Publisher()
        : Node("dr_servo_deg_publisher") {
        // Publisherの作成
        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("dr_servo_deg", 10);

        // タイマーを使って定期的にメッセージをpublish
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&Servo_Deg_Publisher::publish_message, this));
    }

private:
    void publish_message() {
        auto message = std_msgs::msg::Int32MultiArray();
        message.data = {data[7], data[8], data[9], data[10]};

        // RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
        publisher_->publish(message); // メッセージをpublish
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

class Params_Listener : public rclcpp::Node {
public:
    Params_Listener()
        : Node("nr25_dr_servo_cal_listener") {
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "dr_servo_cal", 10,
            std::bind(&Params_Listener::params_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "DR Servo Calibrator Listener");
    }

private:
    void params_listener_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        SERVO1_CAL = msg->data[0];
        SERVO2_CAL = msg->data[1];
        SERVO3_CAL = msg->data[2];
        SERVO4_CAL = msg->data[3];
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

// LD19（LiDAR）から取得した最近傍点までの距離を受信するクラス
class LD19_Listener : public rclcpp::Node {
public:
    LD19_Listener() // ここがコンストラクタ！
        : Node("nhk25_dr_ld19") {
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "nearest_point", 10,
            std::bind(&LD19_Listener::ld19_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "NHK2025 LD19 Listener");
    }

private:
    void ld19_listener_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        min_distance = msg->data[0];
        // std::cout << min_distance << std::endl;
        // 障害物の有無（しきい値は要調整）
        if (min_distance < 1.1) {
            front_cleared = false;
        } else {
            front_cleared = true;
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
};

// ソフト緊停用のクラス,PS4_Listernerと並列してPSボタンを監視
// PS4_Listener内で実装すると割り込みができないため分離
class SoftES_Listener : public rclcpp::Node {
public:
    SoftES_Listener(const std::string &ip, int port)
        : Node("dr_soft_es"), udp_(ip, port) {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy1", 10,
            std::bind(&SoftES_Listener::es_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "NHK2025 DR SD initialized with IP: %s, Port: %d", ip.c_str(),
                    port);
    }

private:
    // コントローラーの入力を取得、使わない入力はコメントアウト推奨
    void es_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

        bool PS = msg->buttons[10];
        // stticが大事！！
        static bool ps_state = false; // トグル状態を保持する変数
        static bool ps_prev = false;  // 前回のPSボタンの状態を記録
        static bool soft_es = false;  // ソフトウェア緊急停止状態を保持する変数

        // PSボタンで緊急停止
        if (PS && !ps_prev) {
            ps_state = !ps_state; // トグル切り替え
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        ps_prev = PS;       // 現在のPSボタンの状態を保存
        soft_es = ps_state; // 緊急停止状態を保存

        if (soft_es) {
            data[0] = -1;
            std::cout << "緊急停止！" << std::endl;
        } else if (MC_PRINTF == 0 || MC_PRINTF == 1) {
            data[0] = MC_PRINTF;
        } else {
            data[0] = 1;
        }
        udp_.send(data);
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    UDP udp_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet DR SwerveDrive";
    int result = std::system(figletout.c_str());
    if (result != 0) {
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        std::cerr << "Please install 'figlet' with the following command:" << std::endl;
        std::cerr << "sudo apt install figlet" << std::endl;
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    }

    rclcpp::executors::MultiThreadedExecutor exec; // マルチスレッドに変更（意味あるかは知らん）
    auto ps4_listener = std::make_shared<PS4_Listener>(IP_DR_SD, PORT_DR_SD);
    auto servo_deg_publisher = std::make_shared<Servo_Deg_Publisher>();
    auto params_listener = std::make_shared<Params_Listener>();
    auto ld19_listener = std::make_shared<LD19_Listener>();
    auto es_listener = std::make_shared<SoftES_Listener>(IP_DR_SD, PORT_DR_SD);
    exec.add_node(ps4_listener);
    exec.add_node(servo_deg_publisher);
    exec.add_node(params_listener);
    exec.add_node(ld19_listener);
    exec.add_node(es_listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}