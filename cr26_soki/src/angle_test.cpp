/*
ros2can (MODE_ROBOMAS) 経由でDJIロボマスの角度制御を行うホスト側テストプログラム
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

// 自作 (common パッケージ)
#include "common/common.hpp"

namespace {

// serial_bridge/ros2can プロトコルの固定スロット数
// [ros2can/ros2can/frame_codec.py の SLOT_COUNT と一致させること]
constexpr int kSlotCount = 24;

}  // namespace

class HardWareControl : public rclcpp::Node
{
public:
    HardWareControl() : Node("angle_test")
    {
        /*
        ros2can (MODE_ROBOMAS) への指令フレーム (robomas.cpp 参照、24スロット):

        送信 (PC -> 本機, serial_tx_[robomas_device_id]、本機側Rx_16Data):
          0-3: target_rpm (モータ1-4、生のrpm値、スケール無し)
          4-23: 未使用

        位置フィードバックはrobomas自身の帰還ではなく、出力軸に直付けされた
        別体のAMTエンコーダ (serial_rx_[encoder_device_id]_unwrapped) のdata[3]を使用する。
        ラップ検出(1周8192カウント、4周=32768カウントで0にリセットされる多回転対応)は
        本ノードでは行わず、既に連続値化されたros2can側の_unwrappedトピック
        (Int32MultiArray)を購読する。

        firmware(robomas.cpp)側にはCAN/シリアル途絶時のフェイルセーフが無く、
        最後に受信したtarget_rpmを保持し続ける(esc_ctrl_node.cppのb-g431-esc1と同様)。
        そのためエンコーダのフィードバックがcommand_timeout_sec以上途絶えた場合や
        本ノード終了時は、本ノード側でtarget_rpmを0にフォールバックする(唯一の安全装置)。
        */

        // ros2can (xiao-esp32-s3_can2io, MODE_ROBOMAS) の DEVICE_ID (指令送信先)
        robomas_device_id_ = declare_parameter<int>("robomas_device_id", 102);
        // 制御対象のロボマスのインデックス (0-3: モータ1-4)
        robomas_motor_index_ = declare_parameter<int>("robomas_motor_index", 0);
        // 位置フィードバック用AMTエンコーダを繋いだ別マイコンのDEVICE_ID (robomasとは別体)
        encoder_device_id_ = declare_parameter<int>("encoder_device_id", 101);
        // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意
        publish_rate_ms_ = declare_parameter<int>("publish_rate_ms", 10);
        const double kp = declare_parameter<double>("kp", 0.25);
        const double kd = declare_parameter<double>("kd", 0.05);
        const double pd_max_out = declare_parameter<double>("pd_max_out", 100.0);
        // 目標rpmの安全クランプ (念のための二重の飽和、PD側のmax_outが本来の制限)
        max_target_rpm_ = std::abs(declare_parameter<double>("max_target_rpm", 300.0));
        // エンコーダのフィードバックがこの秒数以上途絶えたらtarget_rpmを0にする
        const double command_timeout_sec = declare_parameter<double>("command_timeout_sec", 0.5);
        show_info_logs_ = declare_parameter<bool>("show_info_logs", true);

        if (robomas_motor_index_ < 0 || robomas_motor_index_ >= kSlotCount) {
            throw std::runtime_error(
                "robomas_motor_index is out of range (must be within 0-" +
                std::to_string(kSlotCount - 1) + ")");
        }

        command_timeout_ = rclcpp::Duration::from_seconds(command_timeout_sec);

        pd_angle_ = std::make_unique<PDController>(
            static_cast<float>(kp), static_cast<float>(kd), static_cast<float>(pd_max_out));

        // joyノードのSubscribe
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&HardWareControl::ps4_listener_callback, this, std::placeholders::_1));

        // serial_tx_/serial_rx_ は ros2can 側がルート名前空間で作るトピックなので、
        // 本ノードを名前空間付きで起動しても解決先がずれないよう絶対名にする。
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "/serial_tx_" + std::to_string(robomas_device_id_), 10);

        // timer_callbackを呼び出すタイマーを作成
        timer_ = create_wall_timer(
            std::chrono::milliseconds(publish_rate_ms_),
            std::bind(&HardWareControl::publisher_timer_callback, this));

        // AMTエンコーダ(robomasとは別体のマイコン)からのSubscribe
        // ros2can が受信ループ内で既に連続値化(unwrap)して配信するトピックを使う
        // (このノード自身ではラップ検出を行わない。esc_ctrl_node.cpp冒頭コメント参照)
        sensor_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/serial_rx_" + std::to_string(encoder_device_id_) + "_unwrapped",
            10,
            std::bind(&HardWareControl::sensor_callback,
                      this,
                      std::placeholders::_1));

        pd_angle_->set_target(static_cast<float>(target_angle_deg_));
        pd_angle_->reset();

        rclcpp::on_shutdown([this]() { send_zero_and_stop(); });

        RCLCPP_INFO(get_logger(),
                    "angle_test started: serial_tx_%d (motor index %d), encoder: serial_rx_%d, "
                    "command_timeout=%.2fs",
                    robomas_device_id_, robomas_motor_index_, encoder_device_id_,
                    command_timeout_sec);
    }

private:
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {

        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        // float LS_X = -1 * msg->axes[0];
        // float LS_Y = msg->axes[1];
        // float RS_X = -1 * msg->axes[3];
        // float RS_Y = msg->axes[4];

        // bool CROSS = msg->buttons[0];
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

        if (UP && !last_up_)
        {
            target_angle_deg_ += 90.0;
            pd_angle_->set_target(static_cast<float>(target_angle_deg_));
            pd_angle_->reset();
            // Initialize internal last_current_ to avoid a large derivative spike
            pd_angle_->update(static_cast<float>(enc1_total_angle_deg_), 1.0f);
        }

        if (DOWN && !last_down_)
        {
            target_angle_deg_ -= 90.0;
            pd_angle_->set_target(static_cast<float>(target_angle_deg_));
            pd_angle_->reset();
            // Initialize internal last_current_ to avoid a large derivative spike
            pd_angle_->update(static_cast<float>(enc1_total_angle_deg_), 1.0f);
        }
        last_up_ = UP;
        last_down_ = DOWN;
        // 配列操作ここまで
    }

    // publish
    void publisher_timer_callback()
    {
        // エンコーダのフィードバックが途絶えたら target_rpm を 0 にフォールバックする
        // (firmware側にはこの種のタイムアウトが無いため、唯一の安全装置)
        const bool stale = !has_feedback_ || (now() - last_feedback_stamp_) > command_timeout_;
        if (stale)
        {
            if (!was_stale_)
            {
                pd_angle_->reset();
            }
            //target_rpm_command_ = 0;
            was_stale_ = true;
        }

        std_msgs::msg::Int16MultiArray msg;
        msg.data.assign(kSlotCount, 0);
        msg.data[robomas_motor_index_] = target_rpm_command_;

        publisher_->publish(msg);

        if (show_info_logs_)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 500,
                "target_angle=%.1fdeg angle=%.1fdeg target_rpm=%d stale=%d",
                target_angle_deg_, enc1_total_angle_deg_, target_rpm_command_, stale);
        }
    }

    void sensor_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        if (static_cast<int>(msg->data.size()) <= 3)
        {
            return;
        }

        const rclcpp::Time current_time = now();
        if (!has_feedback_)
        {
            last_control_time_ = current_time;
        }
        has_feedback_ = true;
        last_feedback_stamp_ = current_time;

        // ros2can側で既に連続値化(unwrap)済みの生カウント (1周8192カウント)。
        // 自前でのラップ検出はここでは行わない(esc_ctrl_node.cpp冒頭コメント参照)。
        const int64_t enc1_total_encoder = msg->data[3];
        const double ENCODER_MAX = 8192.0;    // エンコーダ1周あたりのカウント数
        enc1_total_angle_deg_ = static_cast<double>(enc1_total_encoder) * (360.0 / ENCODER_MAX);

        if (!target_initialized_)
        {
            target_angle_deg_ = enc1_total_angle_deg_;
            pd_angle_->set_target(static_cast<float>(target_angle_deg_));
            pd_angle_->reset();
            // prime last_current_ to avoid derivative spike on first update
            pd_angle_->update(static_cast<float>(enc1_total_angle_deg_), 1.0f);
            target_initialized_ = true;
        }

        // dtの下限をクランプ (エンコーダマイコンからのメッセージ間隔が数msまで詰まることがあり、
        // 微分項が (current-last)/dt で跳ね上がって振動の原因になるため)
        const double dt = std::max((current_time - last_control_time_).seconds(), 0.005);
        last_control_time_ = current_time;

        if (was_stale_)
        {
            // 途絶からの復帰: 微分の跳ねを防ぐためlast_current_をprime
            pd_angle_->update(static_cast<float>(enc1_total_angle_deg_), 1.0f);
            was_stale_ = false;
        }

        // 角度PDの出力をそのまま目標rpmとしてrobomasの速度ループへ渡す
        const float command = pd_angle_->update(static_cast<float>(enc1_total_angle_deg_), static_cast<float>(dt));

        // smooth command to avoid sudden jumps (helps with derivative noise)
        const double smoothed_command = smoothing_alpha_ * prev_command_ + (1.0 - smoothing_alpha_) * static_cast<double>(command);
        prev_command_ = smoothed_command;

        target_rpm_command_ = static_cast<int16_t>(
            std::clamp(smoothed_command, -max_target_rpm_, max_target_rpm_));
        // 受信データ処理ここまで
    }

    void send_zero_and_stop()
    {
        target_rpm_command_ = 0;
        std_msgs::msg::Int16MultiArray msg;
        msg.data.assign(kSlotCount, 0);
        publisher_->publish(msg);
    }

    int robomas_device_id_;
    int robomas_motor_index_;
    int encoder_device_id_;
    int publish_rate_ms_;
    double max_target_rpm_;
    rclcpp::Duration command_timeout_{0, 0};
    bool show_info_logs_ = true;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double enc1_total_angle_deg_ = 0.0;
    bool last_up_ = false;
    bool last_down_ =false;
    bool target_initialized_ = false;
    double target_angle_deg_ = 0.0;
    rclcpp::Time last_control_time_;
    std::unique_ptr<PDController> pd_angle_;

    // smoothing for controller output to prevent rapid jumps
    double prev_command_ = 0.0;
    const float smoothing_alpha_ = 0.7f; // higher -> more smoothing

    int16_t target_rpm_command_ = 0;

    bool has_feedback_ = false;
    bool was_stale_ = true;
    rclcpp::Time last_feedback_stamp_;
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

    auto hardware_control = std::make_shared<HardWareControl>();
    exec.add_node(hardware_control);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
