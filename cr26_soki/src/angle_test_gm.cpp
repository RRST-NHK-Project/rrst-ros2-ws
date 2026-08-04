/*
ros2can (MODE_ROBOMAS) 経由でDJIロボマスの角度制御を行うホスト側テストプログラム
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

#include <chrono>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <thread>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

// 自作 (common パッケージ)
#include "common/common.hpp"

// 以下マイコンに合わせて設定
// ros2can (xiao-esp32-s3_can2io, MODE_ROBOMAS) の DEVICE_ID / CAN_ID (指令送信先)
#define ROBOMAS_DEVICE_ID 101

// 制御対象のロボマスのインデックス (0-3: モータ1-4)
#define ROBOMAS_MOTOR_INDEX 0

// 位置フィードバック用AMTエンコーダを繋いだ別マイコンのDEVICE_ID (robomasとは別体)
#define ENCODER_DEVICE_ID 101

#define ARRAY_SIZE 24 // シリアルフレームのスロット数 (frame_data.hppのTx16NUM/Rx16NUMと同じ)

#define PUBLISH_RATE_MS 10 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

// スティックのデッドゾーン
#define DEADZONE_L 0.3
#define DEADZONE_R 0.3

// 目標rpmの安全クランプ (念のための二重の飽和)
#define ROBOMAS_MAX_TARGET_RPM 300.0

// ===== 角度制御のパラメータ =====
// 最高速度[rpm]。大きくしても下のANGLE_MAX_ACC_DPS2で頭打ちになることが多い。
#define ANGLE_MAX_VEL_RPM 80.0
// 減速に使える加速度[deg/s^2]。応答速度を決める主ノブ。
// 上げる->速い / 実機が追従できないとオーバーシュート、下げる->遅いが確実。
#define ANGLE_MAX_ACC_DPS2 1000.0
// 停止直前(線形域)のゲイン[rpm/deg]。上げると静定が速いが、大きすぎるとハンチング。
#define ANGLE_KP_RPM_PER_DEG 2.0
// 指令が実際に効くまでの実効むだ時間[s] (publish周期+シリアル往復+ファーム周期)。
// この間に進む分を偏差から先読みで差し引くことで、D項の代わりの制動をかける。
#define ANGLE_LOOP_DELAY_S 0.03


class HardWareControl : public rclcpp::Node
{
public:
    HardWareControl(uint8_t robomas_device_id, uint8_t encoder_device_id)
        : Node("hardware_control_" + std::to_string(robomas_device_id)),
          robomas_device_id_(robomas_device_id),
          encoder_device_id_(encoder_device_id)
    {
        /*
        ros2can (MODE_ROBOMAS) への指令フレーム (robomas.cpp 参照、24スロット):

        送信 (PC -> 本機, serial_tx_[ROBOMAS_DEVICE_ID]、本機側Rx_16Data):
          0-3: target_rpm (モータ1-4、生のrpm値、スケール無し)
          4-23: 未使用

        受信 (本機 -> PC, serial_rx_[ENCODER_DEVICE_ID]、本機側Tx_16Data):
          0-3: angle    [0.1deg単位] (出力軸換算、多回転累積済み)
          4-7: velocity [rpm]
          8-11: current [mA]
          12-23: 未使用

        位置フィードバックはrobomas自身の帰還 (data[0]) を使用するため、
        指令先とフィードバック元は同じマイコン (ROBOMAS_DEVICE_ID == ENCODER_DEVICE_ID)。
        */

        // joyノードのSubscribe
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&HardWareControl::ps4_listener_callback, this, std::placeholders::_1));

        // ros2can (robomas) へpublish (target_rpm指令)
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(robomas_device_id_), 10);

        // timer_callbackを呼び出すタイマーを作成
        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&HardWareControl::publisher_timer_callback, this));

        // AMTエンコーダ(robomasとは別体のマイコン)からのSubscribe
        sensor_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(encoder_device_id_),
            10,
            std::bind(&HardWareControl::sensor_callback,
                      this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "serial_tx_%d started (encoder: serial_rx_%d).",
                    robomas_device_id_, encoder_device_id_);
    }

private:
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {

        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        // float LS_X = -1 * msg->axes[0];
        // float LS_Y = msg->axes[1];
        // float RS_X = -1 * msg->axes[3];
        // float RS_Y = msg->axes[4];

        bool CROSS = msg->buttons[0];
        // bool CIRCLE = msg->buttons[1];
        // bool TRIANGLE = msg->buttons[2];
        // bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        // bool L1 = msg->buttons[4];
        // bool R1 = msg->buttons[5];

        // float L2_DIGITAL = (-1 * msg->axes[2] + 1) / 2;
        // float R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2;

        //bool L2 = msg->buttons[6];
        //bool R2 = msg->buttons[7];

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        // static bool last_option = false;
        // static bool option_latch = false;

        // static bool last_share = false;
        // static bool share_latch = false;

        // 目標を書き換えるだけでよい。速度プロファイルは偏差から毎回作り直すので、
        // 移動中に目標が変わっても内部状態のリセットは不要。
        if (UP && !last_up_)
        {
            target_angle_deg_ += 90.0;
        }

        if (DOWN && !last_down_)
        {
            target_angle_deg_ -= 90.0;
        }

        // ×ボタンで0degへ復帰。ここでの0degはファーム起動時に
        // rotation_count=0 とした基準位置 (電源投入時にモータがいた回転内の
        // エンコーダ原点) であり、機構の原点とは限らない点に注意。
        if (CROSS && !last_cross_)
        {
            target_angle_deg_ = 0.0;
        }

        last_up_ = UP;
        last_down_ = DOWN;
        last_cross_ = CROSS;

        RCLCPP_INFO(
            get_logger(),
            "target_angle= %.1f, angle= %.1f, target_rpm= %d",
            target_angle_deg_, enc1_total_angle_deg_, target_rpm_command_
        );
        // 配列操作ここまで
    }

    // publish
    void publisher_timer_callback()
    {
        std_msgs::msg::Int16MultiArray msg;

        msg.data.assign(ARRAY_SIZE, 0);
        msg.data[ROBOMAS_MOTOR_INDEX] = target_rpm_command_;

        publisher_->publish(msg);
    }

    void
    sensor_callback(
        const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {
        // ros2can (MODE_ROBOMAS) の帰還スロット0-3: 出力軸角度 [0.1deg単位]
        // GM6020はダイレクトドライブなのでギア比換算なし。ファーム側で多回転分を
        // 累積済みのため、ここでのラップ検出は不要。
        // int16のため±3276.7deg(約±9回転)を超えると折り返す点に注意。
        const int16_t angle_raw = msg->data[0];

        // 帰還スロット4-7: 出力軸速度 [rpm]。位置を微分せずにこれを使う。
        const double vel_rpm = static_cast<double>(msg->data[4 + ROBOMAS_MOTOR_INDEX]);

        enc1_total_angle_deg_ = static_cast<double>(angle_raw) * 0.1;

        if (!target_initialized_)
        {
            target_angle_deg_ = enc1_total_angle_deg_;
            target_initialized_ = true;
        }

        // むだ時間の間に進んでしまう分を先読みして偏差から差し引く。位置の差分では
        // なく実測速度から作るので、受信間隔のばらつき(dt)に一切影響されない。
        // これがD項の代わりの制動になる。
        const double error_deg = (target_angle_deg_ - enc1_total_angle_deg_) - vel_rpm * 6.0 * ANGLE_LOOP_DELAY_S;
        const double abs_error = std::abs(error_deg);

        // 内側(ファーム)が速度ループなので、外側は目標速度を作るだけでよい。
        // 指令速度は次の3つの最小値:
        //   1) 最高速度
        //   2) sqrt(2*a*|e|)  : 今の偏差から減速して止まりきれる速度 (制動曲線)
        //   3) Kp*|e|         : 停止直前の線形域 (2)は原点で傾きが無限大になるため)
        // 2)があるので、どれだけ大きなステップを与えてもオーバーシュートしない。
        const double command = std::copysign(
            std::min({ANGLE_MAX_VEL_RPM,
                      std::sqrt(2.0 * ANGLE_MAX_ACC_DPS2 * abs_error) / 6.0, // deg/s -> rpm
                      ANGLE_KP_RPM_PER_DEG * abs_error}),
            error_deg);

        // 指令のがたつきを抑える一次フィルタ。微分をやめたので強くかける必要はなく、
        // 大きくすると応答が遅れるだけになる。
        const double smoothed_command = smoothing_alpha_ * prev_command_ + (1.0 - smoothing_alpha_) * command;
        prev_command_ = smoothed_command;

        // 指令はint16(1rpm刻み)なので、切り捨てにすると |偏差| < 1/Kp [deg] で
        // 指令が0になる不感帯ができる。四捨五入にして不感帯を半分にする。
        const double clamped_command =
            std::clamp(smoothed_command, -ROBOMAS_MAX_TARGET_RPM, ROBOMAS_MAX_TARGET_RPM);
        target_rpm_command_ = static_cast<int16_t>(std::lround(clamped_command));

        // const double error_deg = target_angle_deg_ - angle_deg;
        // RCLCPP_INFO(get_logger(), "angle: %.3f deg target: %.3f deg err: %.3f cmd: %.3f smoothed: %.3f target_rpm: %d",
        //         angle_deg, target_angle_deg_, error_deg, static_cast<double>(command), smoothed_command, target_rpm_command_);
        // 受信データ処理ここまで
    }

    uint8_t robomas_device_id_;
    uint8_t encoder_device_id_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double enc1_total_angle_deg_ = 0.0;
    bool last_up_ = false;
    bool last_down_ =false;
    bool last_cross_ = false;
    bool target_initialized_ = false;
    double target_angle_deg_ = 0.0;

    // 指令の一次フィルタ (大きいほど平滑だが遅れる)
    double prev_command_ = 0.0;
    const double smoothing_alpha_ = 0.2;

    int16_t target_rpm_command_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet ! Welcome Day !";
    int result = std::system(figletout.c_str());
    if (result != 0)
    {
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
        std::cerr << "Please install 'figlet' with the following command:"
                  << std::endl;
        std::cerr << "sudo apt install figlet" << std::endl;
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
    }

    rclcpp::executors::MultiThreadedExecutor exec;

    auto hardware_control = std::make_shared<HardWareControl>(ROBOMAS_DEVICE_ID, ENCODER_DEVICE_ID);
    exec.add_node(hardware_control);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
