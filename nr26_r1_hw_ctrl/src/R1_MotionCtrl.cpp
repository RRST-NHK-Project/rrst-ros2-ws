/*
R1機構制御
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

// 以下マイコンに合わせて設定
#define OUTPUT_DEVICE_ID 2 // 出力マイコン（モーター制御）のID
#define INPUT_DEVICE_ID  3 // 入力マイコン（マイクロスイッチ）のID
#define TX16NUM 24         // 送信データ数
#define RX16NUM 17         // 受信データ数

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

// スティックのデッドゾーン
#define DEADZONE_L 0.3
#define DEADZONE_R 0.3

// =================================================================
// マイクロスイッチの状態（ID=3のESP32から受信、2ノード間で共有）
// atomic: スレッドセーフに読み書きするため
// =================================================================
std::atomic<int16_t> g_micro1_sw{0}; // マイクロスイッチ(上): 1=押されている
std::atomic<int16_t> g_micro2_sw{0}; // マイクロスイッチ(下): 1=押されている

// =================================================================
// SwitchInputノード: ID=3のESP32からマイクロスイッチの状態を受信する
// =================================================================
class SwitchInput : public rclcpp::Node {
public:
    SwitchInput()
        : Node("switch_input_" + std::to_string(INPUT_DEVICE_ID)) {

        sw_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(INPUT_DEVICE_ID),
            10,
            std::bind(&SwitchInput::sw_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "SwitchInput: serial_rx_%d を受信開始", INPUT_DEVICE_ID);
    }

private:
    void sw_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        // ESP32 MODE=3 (8個) や直接通信の場合に対応するため、チェックを緩和
        if (msg->data.size() < 6) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "serial_rx_%d: データが短すぎます (%zu)",
                        INPUT_DEVICE_ID, msg->data.size());
            return;
        }

        // ESP32 esp32_serial_bridge の送信インデックス:
        //   data[9]   = SW1 (マイクロスイッチ 上)
        //   data[10]  = SW2 (マイクロスイッチ 下)
        g_micro1_sw = msg->data[9]; 
        g_micro2_sw = msg->data[10]; 

        // デバッグ: 受信した全てのデータを表示（インデックス確認用）
        std::string debug_str = "[ID " + std::to_string(INPUT_DEVICE_ID) + " RX] ";
        for (size_t i = 0; i < msg->data.size(); i++) {
            debug_str += "d[" + std::to_string(i) + "]=" + std::to_string(msg->data[i]) + " ";
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", debug_str.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sw_sub_;
};

// =================================================================
// HardWareControlノード: ID=2のESP32へモーター指令を送信する
// =================================================================
class HardWareControl : public rclcpp::Node {
public:
    HardWareControl()
        : Node("hardware_control_" + std::to_string(OUTPUT_DEVICE_ID)) {

        // 配列を0で初期化
        data_.assign(TX16NUM, 0);
        /*
        マイコンに送信される配列"data_"
        debug: 機能未割り当て, MD: モータードライバー, TR: トランジスタ
        | data[n] | 詳細 | 範囲 |
        | ---- | ---- | ---- |
        | data[0] | debug | 0 or 1 |
        | data[1] | MD1 | -100 ~ 100 |
        | data[2] | MD2 | -100 ~ 100 |
        | data[3] | MD3 | -100 ~ 100 |
        | data[4] | MD4 | -100 ~ 100 |
        | data[5] | MD5 | -100 ~ 100 |
        | data[6] | MD6 | -100 ~ 100 |
        | data[7] | MD7 | -100 ~ 100 |
        | data[8] | MD8 | -100 ~ 100 |
        | data[9] | Servo1 | 0 ~ 270 |
        | data[10] | Servo2 | 0 ~ 270 |
        | data[11] | Servo3 | 0 ~ 270 |
        | data[12] | Servo4 | 0 ~ 270 |
        | data[13] | Servo5 | 0 ~ 270 |
        | data[14] | Servo6 | 0 ~ 270 |
        | data[15] | Servo7 | 0 ~ 270 |
        | data[16] | Servo8 | 0 ~ 270 |
        | data[17] | TR1 | 0 or 1|
        | data[18] | TR2 | 0 or 1|
        | data[19] | TR3 | 0 or 1|
        | data[20] | TR4 | 0 or 1|
        | data[21] | TR5 | 0 or 1|
        | data[22] | TR6 | 0 or 1|
        | data[23] | TR7 | 0 or 1|
        | data[24] | TR8 | 0 or 1|
        */

        // joyノードのSubscribe
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&HardWareControl::ps4_listener_callback, this, std::placeholders::_1));

        // seial_bridgeへpublish
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(OUTPUT_DEVICE_ID), 10);

        // timer_callbackを呼び出すタイマーを作成
        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&HardWareControl::publisher_timer_callback, this));

        // sensor_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        //     "serial_rx_" + std::to_string(device_id_),
        //     10,
        //     std::bind(&HardWareControl::sensor_callback,
        //               this,
        //               std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "HardWareControl: serial_tx_%d 送信開始", OUTPUT_DEVICE_ID);
    }

private:
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        // float LS_X = -1 * msg->axes[0];
        // float LS_Y = msg->axes[1];
        // float RS_X = -1 * msg->axes[3];
        // float RS_Y = msg->axes[4];

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

        // bool L2 = msg->buttons[6];
        // bool R2 = msg->buttons[7];

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        // static bool last_option = false;
        // static bool option_latch = false;

        // static bool last_share = false;
        // static bool share_latch = false;

        // マイクロスイッチの状態をグローバル変数から取得
        int16_t micro1_sw = g_micro1_sw.load(); 
        int16_t micro2_sw = g_micro2_sw.load(); 

        // 制御ノード側のデバッグログ
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "【制御ノード表示】SW状態: 上=%d (%s), 下=%d (%s)", 
            micro1_sw, micro1_sw ? "停止条件成立" : "通常",
            micro2_sw, micro2_sw ? "停止条件成立" : "通常");

        // 以降、配列data_を操作する
        // =================================================================
        // CROSS:「マガジン回転」
        // ボタンを一回押すごとにサーボモーターを45°づつ回転する
        // =================================================================

        static int MAG_SERVO_ANGLE[] = {270, 228, 186, 146, 100, 58, 13, 141, 95, 50, 53, 5, 8};
        static int cross_pre = 0;
        static int CROSS_PUSH_COUNT = 0;
        static int CROSS_PUSH_MAX = 13;

        if (CROSS == 1 && cross_pre == 0) {
            CROSS_PUSH_COUNT = (CROSS_PUSH_COUNT + 1) % CROSS_PUSH_MAX;
        }
        if (CROSS_PUSH_COUNT == 0) {
            data_[9] = MAG_SERVO_ANGLE[0];
        }
        if (CROSS_PUSH_COUNT == 1) {
            data_[9] = MAG_SERVO_ANGLE[1];
        }
        if (CROSS_PUSH_COUNT == 2) {
            data_[9] = MAG_SERVO_ANGLE[2];
        }
        if (CROSS_PUSH_COUNT == 3) {
            data_[9] = MAG_SERVO_ANGLE[3];
        }
        if (CROSS_PUSH_COUNT == 4) {
            data_[9] = MAG_SERVO_ANGLE[4];
        }
        if (CROSS_PUSH_COUNT == 5) {
            data_[9] = MAG_SERVO_ANGLE[5];
        }
        if (CROSS_PUSH_COUNT == 6) {
            data_[9] = MAG_SERVO_ANGLE[6];
        }
        if (CROSS_PUSH_COUNT == 7) {
            data_[9] = MAG_SERVO_ANGLE[7];
        }
        if (CROSS_PUSH_COUNT == 8) {
            data_[9] = MAG_SERVO_ANGLE[8];
        }
        if (CROSS_PUSH_COUNT == 9) {
            data_[9] = MAG_SERVO_ANGLE[9];
        }
        if (CROSS_PUSH_COUNT == 10) {
            data_[9] = MAG_SERVO_ANGLE[10];
        }
        if (CROSS_PUSH_COUNT == 11) {
            data_[9] = MAG_SERVO_ANGLE[11];
        }
        if (CROSS_PUSH_COUNT == 12) {
            data_[9] = MAG_SERVO_ANGLE[12];
        }
        cross_pre = CROSS;

        // =================================================================
        // CIRCLE:「棒ホールド機構」
        // ボタンを一回押すごとに2つのサーボモーターの角度状態を同時に変化させる
        // =================================================================
        // angle = 10のとき最下部までお仕込み
        // angle = 245のときマガジンに戻してる

        static int BAR_PUSH_ANGLE = 10;
        static int BAR_HOLD_ANGLE = 43;
        static int BAR_RELE_ANGLE = 245;
        static int circle_pre = 0;
        static int CIRCLE_PUSH_COUNT = 0;
        static int CIRCLE_PUSH_MAX = 3;

        if (CIRCLE == 1 && circle_pre == 0) {
            CIRCLE_PUSH_COUNT = (CIRCLE_PUSH_COUNT + 1) % CIRCLE_PUSH_MAX;
        }
        if (CIRCLE_PUSH_COUNT == 0) {
            data_[10] = BAR_RELE_ANGLE;
        }
        if (CIRCLE_PUSH_COUNT == 1) {
            data_[10] = BAR_HOLD_ANGLE;
        }
        if (CIRCLE_PUSH_COUNT == 2) {
            data_[10] = BAR_PUSH_ANGLE;
        }

        circle_pre = CIRCLE;

        // =================================================================
        // TRIANGLE:「棒取捨選択」
        //
        // =================================================================

        static int triangle_pre = 0;
        static int tri_count = 0;
        static int tri_state;
        static int ang_1 = 13;
        static int ang_2 = 90;

        if (TRIANGLE == 1 && triangle_pre == 0)
            tri_count++;

        tri_state = tri_count % 2;

        if (tri_state == 0) {
            data_[12] = ang_1;
        } else if (tri_state == 1) {
            data_[12] = ang_2;
        }

        triangle_pre = TRIANGLE;

        // =================================================================
        // SQUARE:「棒ロック」
        // ボタンを一回押すごとに2つのサーボモーターの角度状態を同時に変化させる
        // =================================================================

        static int BAR_BTM_HOLD_ANGLE = 0;
        static int BAR_BTM_ANGLE = 150;
        static int square_pre = 0;
        static int u = 0;
        static int SQUARE_PUSH_MAX = 2;

        if (SQUARE == 1 && square_pre == 0) {
            u = (u + 1) % SQUARE_PUSH_MAX;
        }
        if (u == 0) {
            data_[11] = BAR_BTM_ANGLE;
        }
        if (u == 1) {
            data_[11] = BAR_BTM_HOLD_ANGLE;
        }

        square_pre = SQUARE;

        // =================================================================
        // UP,DOWN:「槍押上機構」
        // ボタンを押し続けると槍を押し上げるモーターが回転し続ける
        // =================================================================

        // data_[3] = UP;
        if (UP) {
            data_[1] = 50;
        } else if (DOWN) {
            data_[1] = -50;
        } else {
            data_[1] = 0;
        }

        // =================================================================
        // L1,R1:「フォークリフト上下」
        // マイクロスイッチによる安全機構付き（ID=3のESP32から受信）
        // マイクロスイッチ(上)押下時: L1による正回転禁止、R1による逆回転は許可
        // マイクロスイッチ(下)押下時: R1による逆回転禁止、L1による正回転は許可
        // =================================================================

        // マイクロスイッチ(上): 押下時は正回転(L1)を禁止、逆回転(R1)のみ許可
        // マイクロスイッチ(下): 押下時は逆回転(R1)を禁止、正回転(L1)のみ許可
        if (L1 == 1 && micro1_sw == 0) {
            // L1ボタンで正回転（マイクロスイッチ(上)が押されていない場合のみ）
            data_[2] = 127;
        } else if (R1 == 1 && micro2_sw == 0) {
            // R1ボタンで逆回転（マイクロスイッチ(下)が押されていない場合のみ）
            data_[2] = -127;
        } else if (R1 == 1 && micro1_sw == 1) {
            // マイクロスイッチ(上)が押されていてもR1による逆回転は許可
            data_[2] = -127;
        } else if (L1 == 1 && micro2_sw == 1) {
            // マイクロスイッチ(下)が押されていてもL1による正回転は許可
            data_[2] = 127;
        } else {
            // どちらのボタンも押されていない場合は停止
            data_[2] = 0;
        }
        // ※この場合、SW1が押されているとき、R1が効かない可能性がある
        // ※また、AW2においても押されているとき、L1が効かない可能性があるので確認すべし

        // =================================================================
        // LR
        // マガジン調整用
        // static int MAG_ADJ_STEP = 5;
        // static int left_pre = 0;
        // static int right_pre = 0;
        // static int mag_servo_angle = 270;

        // if (LEFT == 1 && left_pre == 0) {
        //     mag_servo_angle -= MAG_ADJ_STEP;
        //     data_[9] = mag_servo_angle;
        // }
        // left_pre = LEFT;
        // if (RIGHT == 1 && right_pre == 0) {
        //     mag_servo_angle += MAG_ADJ_STEP;
        //     data_[9] = mag_servo_angle;
        // }
        // right_pre = RIGHT;
        // =================================================================

        // デバッグ用
        RCLCPP_INFO(
            get_logger(),
            "data_[1-4]=[%d,%d,%d,%d], data_[9-12]=[%d,%d,%d,%d]",
            data_[1], data_[2], data_[3], data_[4],
            data_[9], data_[10], data_[11], data_[12]);

        // 配列操作ここまで
    }

    // publish
    void publisher_timer_callback() {
        std_msgs::msg::Int16MultiArray msg;

        msg.data = data_;

        publisher_->publish(msg);
    }

    // void
    // sensor_callback(
    //     const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    //     // 最低限：サイズチェック
    //     if (msg->data.size() < RX16NUM) {
    //         RCLCPP_WARN(this->get_logger(),
    //                     "serial_rx_%d: data too short (%zu)",
    //                     device_id_, msg->data.size());
    //         return;
    //     }

        // int16_t ENC1 = msg->data[1];
        // int16_t ENC2 = msg->data[2];
        // int16_t ENC3 = msg->data[3];
        // int16_t ENC4 = msg->data[4];
        // int16_t ENC5 = msg->data[5];
        // int16_t ENC6 = msg->data[6];
        // int16_t ENC7 = msg->data[7];
        // int16_t ENC8 = msg->data[8];

        // int16_t SW1 = msg->data[9];
        // int16_t SW2 = msg->data[10];
        // int16_t SW3 = msg->data[11];
        // int16_t SW4 = msg->data[12];
        // int16_t SW5 = msg->data[13];
        // int16_t SW6 = msg->data[14];
        // int16_t SW7 = msg->data[15];
        // int16_t SW8 = msg->data[16];

        // 以降、受信データを使った処理を記述
        // エンコーダースイッチの状態を保存（モーター制御で使用）
        // micro1_sw_ = SW1;
        // micro2_sw_ = SW2;

        // デバッグ: マイクロスイッチの受信値を確認
        // RCLCPP_INFO(get_logger(),
        //             "[マイクロSW] 上(L1禁止用)=%s  下(R1禁止用)=%s",
        //             micro1_sw_ ? "★押されている" : "　押されていない",
        //             micro2_sw_ ? "★押されている" : "　押されていない");

        // 受信データ処理ここまで
    // }

    uint8_t device_id_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<int16_t> data_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet R1 Motion Ctrl";
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

    // ID=2: モーター出力ノード
    auto hardware_control = std::make_shared<HardWareControl>();
    exec.add_node(hardware_control);

    // ID=3: マイクロスイッチ入力ノード
    auto switch_input = std::make_shared<SwitchInput>();
    exec.add_node(switch_input);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
