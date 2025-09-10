// 標準ライブラリ
#include <chrono>
#include <iostream>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

// --- グローバル設定 ---
// モーターの基本速度 (-100 ~ 100の範囲を想定)
constexpr int MOTOR_SPEED = 80;
// L2/R2トリガーが反応しない領域（誤操作防止）
constexpr float TRIGGER_DEADZONE = 0.1;


// マイコンに送信されるデータ配列
// [θ1, θ2, θ3, θ4, θ5, グリッパー] の指令値を格納
std::vector<int16_t> data(6, 0);
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

class PS4_Listener : public rclcpp::Node {
    public:
        PS4_Listener(const std::string &ip, int port)
            : Node("nhk25_mr_sd"), udp_(ip, port) {
            subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy", 10,
                std::bind(&PS4_Listener::ps4_listener_callback, this,
                          std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(),
                        "NHK2025 MR SD initialized with IP: %s, Port: %d", ip.c_str(),
                        port);
        }
    
class ArmControllerNode : public rclcpp::Node {
public:
    ArmControllerNode() : Node("arm_controller_node") {
        // "/joy" トピックを購読し、コントローラー入力があるたびに joy_callback を呼ぶ
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&ArmControllerNode::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "アーム制御ノードを起動しました。PSボタンで制御を開始してください。");
    }

private:
    // コントローラー入力があるたびに実行される関数
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // --- 1. コントローラー入力の読み取りと整形 ---
        // 各ボタンの状態を分かりやすい変数に格納
        bool SQUARE   = msg->buttons[3];
        bool CIRCLE   = msg->buttons[1];
        bool TRIANGLE = msg->buttons[2];
        bool CROSS    = msg->buttons[0];

        bool L1 = msg->buttons[4];
        bool R1 = msg->buttons[5];
        
        bool SHARE  = msg->buttons[8];
        bool PS     = msg->buttons[10];

        // 十字キー (アナログ軸として入力される)
        bool UP    = msg->axes[7] == 1.0;
        bool DOWN  = msg->axes[7] == -1.0;
        bool LEFT  = msg->axes[6] == 1.0;
        bool RIGHT = msg->axes[6] == -1.0;

        // L2/R2トリガー (-1.0 ~ 1.0 の値を 0.0 ~ 1.0 に変換)
        float L2 = (msg->axes[2] + 1.0) / 2.0;
        float R2 = (msg->axes[5] + 1.0) / 2.0;

        // デッドゾーン以下のトリガー入力を無視
        if (L2 <= TRIGGER_DEADZONE) L2 = 0.0;
        if (R2 <= TRIGGER_DEADZONE) R2 = 0.0;

        // デッドゾーン以下のトリガー入力を無視
        if (L2 <= TRIGGER_DEADZONE) {
            RCLCPP_DEBUG(this->get_logger(), "L2トリガーがデッドゾーン以下: %f", L2);
            L2 = 0.0;
        }
        if (R2 <= TRIGGER_DEADZONE) {
            RCLCPP_DEBUG(this->get_logger(), "R2トリガーがデッドゾーン以下: %f", R2);
            R2 = 0.0;
        }

        // --- 2. PSボタンによる起動/非常停止 (トグル) ---
        if (PS && !last_ps_state_) {
            control_active_ = !control_active_;
            if (control_active_) {
                RCLCPP_INFO(this->get_logger(), "制御開始 (PSボタン ON)");
            } else {
                RCLCPP_WARN(this->get_logger(), "非常停止 (PSボタン OFF)");
            }
        }
        RCLCPP_DEBUG(this->get_logger(), "PSボタン状態: %d -> %d", last_ps_state_, PS);
        last_ps_state_ = PS;


        // 停止状態なら、全モーター指令を0にして処理を中断
        if (!control_active_) {
            std::fill(data.begin(), data.end(), 0);
            // ここでUDP送信などの処理を入れる
            // udp_.send(data);
            return;
        }


        // --- 3. 各軸の制御ロジック ---
        // θ1 (旋回): 十字キー左右
        if (RIGHT)      data[0] = MOTOR_SPEED;
        else if (LEFT)  data[0] = -MOTOR_SPEED;
        else            data[0] = 0;
        
        // θ2: 未割り当てのため常に0
        data[1] = 0;

        // θ3 (昇降): L1 (上) / L2 (下)
        if (L1)         data[2] = MOTOR_SPEED;
        else if (L2 > 0)data[2] = -MOTOR_SPEED;
        else            data[2] = 0;

        // θ4 (手首上下): R1 (上) / R2 (下)
        if (R1)         data[3] = MOTOR_SPEED;
        else if (R2 > 0)data[3] = -MOTOR_SPEED;
        else            data[3] = 0;

        // θ5 (手首回転): 十字キー上下
        if (UP)         data[4] = MOTOR_SPEED;
        else if (DOWN)  data[4] = -MOTOR_SPEED;
        else            data[4] = 0;


        // --- 4. グリッパーとその他機能 ---
        // グリッパー: 〇ボタン (トグル)
        if (CIRCLE && !last_circle_state_) {
            gripper_closed_ = !gripper_closed_; // 状態を反転
        }
        data[5] = gripper_closed_ ? 1 : 0; // 状態に応じて指令値を設定 (1:閉, 0:開)
        last_circle_state_ = CIRCLE;

        // SHAREボタン

        // --- 5. 指令の送信 (デバッグ表示) ---
        RCLCPP_INFO(this->get_logger(), "送信データ: [θ1:%4d, θ3:%4d, θ4:%4d, θ5:%4d, Grip:%d]",
                    data[0], data[2], data[3], data[4], data[5]);
    }   

    // メンバ変数
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    
    // 状態を保持するための変数
    bool control_active_ = false; // 制御が有効か (PSボタンでトグル)
    bool gripper_closed_ = false; // グリッパーが閉じているか (〇ボタンでトグル)

    // ボタンが押された瞬間を検知するための変数
    bool last_ps_state_ = false;
    bool last_circle_state_ = false;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet APOLLO4SHOOT";
    int result = std::system(figletout.c_str());
    if (result != 0) {
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
        std::cerr << "Please install 'figlet' with the following command:"
                  << std::endl;
        std::cerr << "sudo apt install figlet" << std::endl;
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
    }

    rclcpp::executors::MultiThreadedExecutor exec;
    auto ps4_listener = std::make_shared<PS4_Listener>(IP_AP4_SHOOT, PORT_AP4_SHOOT);
    exec.add_node(ps4_listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}