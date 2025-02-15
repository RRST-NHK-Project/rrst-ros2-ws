/*
RRST NHK2025
汎用機独ステ
*/

#include "include/UDP.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <cmath>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <thread>

// サーボの組み付け時のズレを補正（度数法）
#define SERVO1_CAL 0 // 7
#define SERVO2_CAL 0
#define SERVO3_CAL 0 // 0
#define SERVO4_CAL 0 //-5

// スティックのデッドゾーン
#define DEADZONE_L 0.7
#define DEADZONE_R 0.2
//定数k
#define k 0.05
//PIDパラメータ（チューニングが必要）
#define deg_Kp 0.01      // 角度Pゲイン
#define deg_Ki 0.5        //角度Iゲイン
#define  deg_Kd 0   // 角度Dゲイン
#define  speed_Kp 0      // 速度Pゲイン
#define  speed_Ki 0.2      // 速度Iゲイン
#define  speed_Kd 0        // 速度Dゲイン
#define speed_limit 15
#define deg_limit 360
#define DPAD_SPEED 30  // 方向パッド入力時の目標速度
//角度一覧
int deg;
int previous_deg =135;
int truedeg;
int desired_deg;
int measured_deg = 0;



//PID用の内部変数
double deg_Error = 0.0;
double deg_last_Error = 0.0;
double speed_Error = 0.0;
double speed_last_Error = 0.0;
double deg_Integral = 0.0;
double deg_Differential = 0.0;
double deg_Output = 0.0;
double speed_Integral = 0.0;
double speed_Differential = 0.0;
double speed_Output = 0.0;

// 制御周期 [秒]
const double dt = 0.01;  // 10ms

//速度
int wheelspeed = 30;
int yawspeed = 10;
int previous_speed = 0;
int desired_speed;
int measured_speed = 0;
static double current_motor_command = 0.0;
//角度の変位
int displacement;

// IPアドレスとポートの指定
std::string udp_ip = "192.168.8.215"; // 送信先IPアドレス、宛先マイコンで設定したIPv4アドレスを指定
int udp_port = 5000;                  // 送信元ポート番号、宛先マイコンで設定したポート番号を指定

std::vector<int> data = {0, 0, 0, 0, 0, 0, 0, 0, 0};


class PS4_Listener : public rclcpp::Node {
public:
    PS4_Listener(const std::string &ip, int port)
        : Node("nhk25_mr_sd"), udp_(ip, port) {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy0", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "NHK2025 MR SD initialized with IP: %s, Port: %d", ip.c_str(),
                    port);
    }

private:
    // コントローラーの入力を取得、使わない入力はコメントアウト推奨
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
        //float R2 = (-1 * msg->axes[5] + 1) / 2;

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
        // float s = sin(rad);
        // float c = cos(rad);
        deg = rad * 180 / M_PI;

        // PID計算（台形則による積分計算をそのまま使用）
        speed_Error = desired_speed - measured_speed;
        
        speed_Integral += (speed_Error + speed_last_Error) * dt/ 2.0;
        speed_Differential = (speed_Error - speed_last_Error) / dt;
        // 各サンプリングごとにPID出力を再計算
        speed_Output = (speed_Kp * speed_Error) + (speed_Ki * speed_Integral) + (speed_Kd * speed_Differential);
        speed_last_Error = speed_Error;

        // PID計算（台形則による積分計算をそのまま使用）
        deg_Error = deg - previous_speed;
        
        deg_Integral += (deg_Error + deg_last_Error) * dt/ 2.0;
        deg_Differential = (deg_Error - deg_last_Error) / dt;
        // 各サンプリングごとにPID出力を再計算
        deg_Output = (deg_Kp * deg_Error) + (deg_Ki * deg_Integral) + (deg_Kd * deg_Differential);
        deg_last_Error = deg_Error;
        
        // deadzone追加
        if ((fabs(LS_X) <= DEADZONE_R) && (fabs(LS_Y) <= DEADZONE_R) && (fabs(RS_X) <= DEADZONE_L)&&(!LEFT && !RIGHT && !UP && !DOWN)) {
            deg = 135;
            // desired_speed = 0;
            // measured_speed = 0;
            
            speed_Integral = 0;
            //speed_last_Error = 0;
            // for (int i = speed_Output;i>=0;i=i-0.25){
            //     data[1] = i;
            //     data[2] = i;
            //     data[3] = i;
            //     data[4] = i;
            // }
            // deadzone状態なら、現状の指令値から0に向かって徐々に減衰させる
            double decay_rate = 0.1; // 0～1の値。大きいほど速く0に近づく
            current_motor_command += (0 - current_motor_command) * decay_rate;
            speed_Output = current_motor_command;
            data[1] = speed_Output;
            data[2] = speed_Output;
            data[3] = speed_Output;
            data[4] = speed_Output;
            data[5] = deg + SERVO1_CAL;
            data[6] = deg + SERVO2_CAL;
            data[7] = deg + SERVO3_CAL;
            data[8] = deg + SERVO4_CAL;
        }else{
            desired_speed = 30;
            current_motor_command = -speed_Output;
        }

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
        // displacement = deg -previous_deg;

        // std::cout << deg << std::endl;

        

        // data[1] = -wheelspeed/(1 + k*displacement) * R2;
        // data[2] = -wheelspeed/(1 + k*displacement) * R2;
        // data[3] = -wheelspeed/(1 + k*displacement) * R2;
        // data[4] = -wheelspeed/(1 + k*displacement) * R2;
        // data[1] = -wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        // data[2] = -wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        // data[3] = -wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        // data[4] = -wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));

        // 安全のため、出力値に対して上限を設ける
        if (speed_Output >= speed_limit) {
            speed_Output = speed_limit;
        } else if (speed_Output < -speed_limit) {
            speed_Output = -speed_limit;
        }
        data[1] = -speed_Output;
        data[2] = -speed_Output;
        data[3] = -speed_Output;
        data[4] = -speed_Output;
        
        // 方向パッド入力による加速処理
        if (LEFT) {
            double targetSpeed = -DPAD_SPEED;
            double decay_rate = 0.1;  // 0～1の値。大きいほど早く目標速度に近づく
            current_motor_command += (targetSpeed - current_motor_command) * decay_rate;
            speed_Output = current_motor_command;
            deg = 45;
        }else if (RIGHT) {
            double targetSpeed = DPAD_SPEED;
            double decay_rate = 0.1;
            current_motor_command += (targetSpeed - current_motor_command) * decay_rate;
            speed_Output = current_motor_command;
            deg = 45;
        }else if (UP) {
            double targetSpeed = -DPAD_SPEED;
            double decay_rate = 0.1;
            current_motor_command += (targetSpeed - current_motor_command) * decay_rate;
            speed_Output = current_motor_command;
            deg = 135;
        }else if (DOWN) {
            double targetSpeed = DPAD_SPEED;
            double decay_rate = 0.1;
            current_motor_command += (targetSpeed - current_motor_command) * decay_rate;
            speed_Output = current_motor_command;
            deg = 135;
        }else if ((fabs(LS_X) <= DEADZONE_R) &&(fabs(LS_Y) <= DEADZONE_R) &&(fabs(RS_X) <= DEADZONE_L) &&(!LEFT && !RIGHT && !UP && !DOWN)){
    // 方向パッドもジョイスティックも入力がない場合は徐々に0に戻す
            double decay_rate = 0.1;
            current_motor_command += (0 - current_motor_command) * decay_rate;
            speed_Output = current_motor_command;
            deg = 135;
        }

        // if (LEFT) {
        //     deg = 45;
        //     //displacement = deg -previous_deg;
        //     // data[1] = -wheelspeed/(1 + k*displacement) * R2;
        //     // data[2] = -wheelspeed/(1 + k*displacement) * R2;
        //     // data[3] = -wheelspeed/(1 + k*displacement) * R2;
        //     // data[4] = -wheelspeed/(1 + k*displacement) * R2;
        //     // data[1] = -wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        //     // data[2] = -wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        //     // data[3] = -wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        //     // data[4] = -wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
            
        //     data[1] = -speed_Output;
        //     data[2] = -speed_Output;
        //     data[3] = -speed_Output;
        //     data[4] = -speed_Output;
        // }
        // if (RIGHT) {
        //     deg = 45;
        //     //displacement = deg -previous_deg;
        //     // data[1] = wheelspeed/(1 + k*displacement) * R2;
        //     // data[2] = wheelspeed/(1 + k*displacement) * R2;
        //     // data[3] = wheelspeed/(1 + k*displacement) * R2;
        //     // data[4] = wheelspeed/(1 + k*displacement) * R2;
        //     // data[1] = wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        //     // data[2] = wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        //     // data[3] = wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        //     // data[4] = wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
            
        //     data[1] = speed_Output;
        //     data[2] = speed_Output;
        //     data[3] = speed_Output;
        //     data[4] = speed_Output;
        // }
        // if (UP) {
        //     deg = 135;
        //     //displacement = deg -previous_deg;
        //     // data[1] = -wheelspeed/(1 + k*displacement) * R2;
        //     // data[2] = -wheelspeed/(1 + k*displacement) * R2;
        //     // data[3] = -wheelspeed/(1 + k*displacement) * R2;
        //     // data[4] = -wheelspeed/(1 + k*displacement) * R2;
        //     // data[1] = -wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        //     // data[2] = -wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        //     // data[3] = -wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        //     // data[4] = -wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
            
        //     data[1] = -speed_Output;
        //     data[2] = -speed_Output;
        //     data[3] = -speed_Output;
        //     data[4] = -speed_Output;
        // }
        // if (DOWN) {
        //     deg = 135;
        //     //displacement = deg -previous_deg;
        //     // data[1] = wheelspeed/(1 + k*displacement) * R2;
        //     // data[2] = wheelspeed/(1 + k*displacement) * R2;
        //     // data[3] = wheelspeed/(1 + k*displacement) * R2;
        //     // data[4] = wheelspeed/(1 + k*displacement) * R2;
        //     // data[1] = wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        //     // data[2] = wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        //     // data[3] = wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        //     // data[4] = wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
            
        //     data[1] = speed_Output;
        //     data[2] = speed_Output;
        //     data[3] = speed_Output;
        //     data[4] = speed_Output;
        // }

        // 独ステが扱えない範囲の変換
        // if ((345 < deg) && (deg < 360)) {
        //     deg = 13*deg - 4320;
        //     //displacement = deg -previous_deg;
        //     // data[1] = wheelspeed/(1 + k*displacement) * R2;
        //     // data[2] = wheelspeed/(1 + k*displacement) * R2;
        //     // data[3] = wheelspeed/(1 + k*displacement) * R2;
        //     // data[4] = wheelspeed/(1 + k*displacement) * R2;
        //     // data[1] = wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        //     // data[2] = wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        //     // data[3] = wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        //     // data[4] = wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        //     data[1] = speed_Output;
        //     data[2] = speed_Output;
        //     data[3] = speed_Output;
        //     data[4] = speed_Output;
        //     data[5] = deg + SERVO1_CAL;
        //     data[6] = deg + SERVO2_CAL;
        //     data[7] = deg + SERVO3_CAL;
        //     data[8] = deg + SERVO4_CAL;
        // }
        // if ((270 < deg) && (deg < 285)) {
        //     deg = -11*deg + 3240;
        //     //displacement = deg -previous_deg;
        //     // data[1] = wheelspeed/(1 + k*displacement) * R2;
        //     // data[2] = wheelspeed/(1 + k*displacement) * R2;
        //     // data[3] = wheelspeed/(1 + k*displacement) * R2;
        //     // data[4] = wheelspeed/(1 + k*displacement) * R2;
        //     // data[1] = wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        //     // data[2] = wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        //     // data[3] = wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        //     // data[4] = wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
        //     data[1] = speed_Output;
        //     data[2] = speed_Output;
        //     data[3] = speed_Output;
        //     data[4] = speed_Output;
        //     data[5] = deg + SERVO1_CAL;
        //     data[6] = deg + SERVO2_CAL;
        //     data[7] = deg + SERVO3_CAL;
        //     data[8] = deg + SERVO4_CAL;
        // }
        if ((270 < deg) && (deg < 360)) {
            deg = deg - 180;
            //displacement = deg -previous_deg;
            // data[1] = wheelspeed/(1 + k*displacement) * R2;
            // data[2] = wheelspeed/(1 + k*displacement) * R2;
            // data[3] = wheelspeed/(1 + k*displacement) * R2;
            // data[4] = wheelspeed/(1 + k*displacement) * R2;
            // data[1] = wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
            // data[2] = wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
            // data[3] = wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
            // data[4] = wheelspeed/(1 + k*displacement) * (pow(s,2) + pow(c,2));
            data[1] = speed_Output;
            data[2] = speed_Output;
            data[3] = speed_Output;
            data[4] = speed_Output;
            data[5] = deg + SERVO1_CAL;
            data[6] = deg + SERVO2_CAL;
            data[7] = deg + SERVO3_CAL;
            data[8] = deg + SERVO4_CAL;
        }
        data[5] = deg + SERVO1_CAL;
        data[6] = deg + SERVO2_CAL;
        data[7] = deg + SERVO3_CAL;
        data[8] = deg + SERVO4_CAL;

        previous_deg = desired_deg;

        // 安全のため、出力値に対して上限を設ける
        if (deg_Output > deg_limit) {
            deg_Output = deg_limit;
        } else if (deg_Output < -deg_limit) {
            deg_Output = -deg_limit;
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
        // deadzone追加
        // if ((fabs(LS_X) <= DEADZONE_R) && (fabs(LS_Y) <= DEADZONE_R) && (fabs(RS_X) <= DEADZONE_L)) {
        //     deg = 135;
        //     desired_speed = 0;
        //     measured_speed = 0;

        //     //speed_Integral = 0;
        //     //speed_last_Error = 0;
        //     data[1] = speed_Output;
        //     data[2] = speed_Output;
        //     data[3] = speed_Output;
        //     data[4] = speed_Output;
        //     data[5] = deg + SERVO1_CAL;
        //     data[6] = deg + SERVO2_CAL;
        //     data[7] = deg + SERVO3_CAL;
        //     data[8] = deg + SERVO4_CAL;
        // }else{
        //     desired_speed = 30;
        // }
        // デバッグ用
        // std::cout << data[1] << ", " << data[2] << ", " << data[3] << ", " << data[4] << ", ";
        // std::cout << data[5] << ", " << data[6] << ", " << data[7] << ", " << data[8] << ", " << std::endl;

        std::cout << data[1] << std::endl;
        udp_.send(data);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    UDP udp_;
};

class Servo_Deg_Publisher : public rclcpp::Node {
public:
    Servo_Deg_Publisher()
        : Node("servo_deg_publisher") {
        // Publisherの作成
        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("mr_servo_deg", 10);

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

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto ps4_listener = std::make_shared<PS4_Listener>(udp_ip, udp_port);
    auto servo_deg_publisher = std::make_shared<Servo_Deg_Publisher>();
    exec.add_node(ps4_listener);
    exec.add_node(servo_deg_publisher);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
