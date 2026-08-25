/*
ros2can (MODE_CUBEMARS) ノードのホスト側プログラム
CubeMars AKシリーズ(Servo(CAN)モード)を位置制御する使用例。
Ros2CanCubemarsPacketControllerを使用してスロット割当を意識せずに送受信配列へ
アクセスする。
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

// ===== Ros2CanCubemarsPacketController (common/Ros2CanCubemarsPacketController.hpp) 関数一覧 =====
// void setPosition(int motor, double deg)
//   指定モータ(motor: 0-origin, 0~MOTOR_COUNT-1)を位置ループ(control_mode=1)にして
//   角度deg[±3276.7]へ動かす(範囲外degはクランプ、motor範囲外は無視)。
//   アクチュエータ内蔵のクローズドループが追従するのでホスト側PIDは不要。
//
// void setVelocity(int motor, double erpm)
//   指定モータを速度ループ(control_mode=0)にして電気角速度erpm[±327670]で回す。
//   出力軸rpmではなく電気角速度なので注意(換算比はモータ機種依存)。
//
// void stop(int motor) / void stopAll()
//   ゼロ速度指令(その場停止)にする。位置ジャンプは発生しない。
//
// double getPosition(int motor) const   現在位置[deg]を取得する。
// double getVelocity(int motor) const   現在速度[電気角ERPM]を取得する。
// double getCurrent(int motor) const    実測電流[A]を取得する。
// int getTemperature(int motor) const   モータ温度[degC]を取得する。
// int16_t getError(int motor) const     エラーコードを取得する(0=no fault)。
// static const char *errorText(int16_t code)
//   エラーコードを人が読める文字列にする(ログ出力用)。
//
// static int canId(int motor)
//   motor(0-origin)に対応するCAN_IDの既定値(101,102,103,104)を返す。
//
// void updateRx(const std::vector<int16_t> &data)
//   受信したInt16MultiArray相当のdataでrx_配列を更新する(sensor_callback等で使用)。
//
// std::vector<int16_t> toVector() const
//   送信用配列tx_をstd::vectorに変換して取得する(publish直前に使用)。
//
// int16_t &operator[](int index) / const int16_t &operator[](int index) const
//   tx_配列への直接アクセス(グローバルslot index指定、通常はsetPosition等を優先)。
//
// メンバ変数:
//   std::array<int16_t, DATA_SIZE> tx_  送信配列本体 (ROS -> マイコン -> CAN -> 各モータ)
//   std::array<int16_t, DATA_SIZE> rx_  受信配列本体 (各モータ -> CAN -> マイコン -> ROS)
// ================================================================================

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>

// ライブラリ
#include "common/Ros2CanCubemarsPacketController.hpp"

// 以下マイコンに合わせて設定
// TX/RX_DEVICE_ID は CubeMarsドライバマイコン自身のシリアルフレームDEVICE_ID
// (firmware/xiao-esp32-s3_can2io/src/config.hpp の DEVICE_ID)。
// CANバス上の各モータのCAN_ID (101,102,103,104、config.hppのCUBEMARS_MOTOR_ID_n)
// とは別物なので注意。
#define TX_DEVICE_ID 101 // 送信先マイコンのDEVICE_ID
#define RX_DEVICE_ID 101 // 受信先マイコンのDEVICE_ID

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

// 制御対象のモータ (0-origin: 0~3、CAN_ID 101~104に対応)
#define TARGET_MOTOR 0

// スティックのデッドゾーン
#define DEADZONE_L 0.3
#define DEADZONE_R 0.3

class Ros2CanCubemarsControl : public rclcpp::Node {
public:
    Ros2CanCubemarsControl(uint8_t tx_device_id, uint8_t rx_device_id)
        : Node("ros2can_cubemars_" + std::to_string(tx_device_id)),
          tx_device_id_(tx_device_id),
          rx_device_id_(rx_device_id) {

        /*
        ros2canが送受信する配列(Ros2CanCubemarsPacketController::tx_ / rx_)は
        serial_bridge互換の24 x int16スロットだが、MODE_CUBEMARSはMODE_CAN_HOSTと違い
        ノード/スロット分配を行わない「独立デバイス」として、24スロットをそのまま
        CubeMars AKシリーズ最大 MOTOR_COUNT 台分の指令/帰還に割り当てる。

        | slot  | 指令(TX)                  | 帰還(RX)              |
        | ----- | ------------------------- | --------------------- |
        | 0-3   | target (モータ1-4)        | position (0.1deg/LSB) |
        | 4-7   | control_mode (0=速度/1=位置) | speed (10ERPM/LSB)  |
        | 8-11  | 未使用                    | current (0.01A/LSB)   |
        | 12-15 | 未使用                    | temperature (degC)    |
        | 16-19 | 未使用                    | error code            |
        | 20-23 | 未使用                    | 未使用                |

        targetの意味はcontrol_modeに依存する:
          control_mode=0(速度): 電気角速度 10ERPM/LSB
          control_mode=1(位置): 位置 0.1deg/LSB
        setPosition()/setVelocity() が target と control_mode を同時に設定するので、
        通常はこの割当を直接意識する必要はない。

        速度/位置ともアクチュエータ内蔵のクローズドループが指令にそのまま追従するため、
        ロボマスのGM6020のようなホスト側PID (common/PID.hpp) は不要。
        */

        // joyノードのSubscribe
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&Ros2CanCubemarsControl::ps4_listener_callback, this, std::placeholders::_1));

        // ros2canへpublish
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(tx_device_id_), 10);

        // timer_callbackを呼び出すタイマーを作成
        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&Ros2CanCubemarsControl::publisher_timer_callback, this));

        sensor_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(rx_device_id_),
            10,
            std::bind(&Ros2CanCubemarsControl::sensor_callback,
                      this,
                      std::placeholders::_1));

        // firmware側にCAN/シリアル途絶時のフェイルセーフは無く、最後に受信した指令を
        // 保持し続ける。ノード終了時は全モータをゼロ速度に戻す(唯一の安全装置)。
        rclcpp::on_shutdown([this]() { send_zero_and_stop(); });

        RCLCPP_INFO(get_logger(),
                    "serial_tx_%d started.", tx_device_id_);
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
        // bool TRIANGLE = msg->buttons[2];
        // bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        // bool UP = msg->axes[7] == 1.0;
        // bool DOWN = msg->axes[7] == -1.0;

        // bool L1 = msg->buttons[4];
        // bool R1 = msg->buttons[5];

        // float L2_DIGITAL = (-1 * msg->axes[2] + 1) / 2;
        // float R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2;

        // bool L2 = msg->buttons[6];
        // bool R2 = msg->buttons[7];

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        // 以降、Ros2CanCubemarsPacketControllerを操作する
        // 例: LS_Yでモータ1を-180~180degに割り当てる場合
        // ctrlPkt_.setPosition(TARGET_MOTOR, LS_Y * 180.0);
        //
        // 注意: setPosition()を呼んだ瞬間に現在位置から目標位置へ移動を開始する。
        // 下の分岐はボタンを押していない間も135degを指令し続けるので、joyが繋がった
        // 時点でモータが135degまで動く。それが困る場合は else 節を
        // ctrlPkt_.stop(TARGET_MOTOR); に変えるか、else 節自体を消して
        // 直前の位置指令を保持させること(位置ループはその位置を保持し続ける)。

        if (CROSS) {
            ctrlPkt_.setPosition(TARGET_MOTOR, 0.0);
        } else if (CIRCLE) {
            ctrlPkt_.setPosition(TARGET_MOTOR, 180.0);
        } else {
            ctrlPkt_.setPosition(TARGET_MOTOR, 90.0);
        }

        // デバッグ用
        // RCLCPP_INFO(
        //     get_logger(),
        //     "motor%d mode=%d target=%d",
        //     TARGET_MOTOR,
        //     ctrlPkt_.tx_[Ros2CanCubemarsPacketController::SLOT_CONTROL_MODE + TARGET_MOTOR],
        //     ctrlPkt_.tx_[Ros2CanCubemarsPacketController::SLOT_TARGET + TARGET_MOTOR]);

        // 配列操作ここまで
    }

    // publish
    void publisher_timer_callback() {
        std_msgs::msg::Int16MultiArray msg;

        msg.data = ctrlPkt_.toVector();

        publisher_->publish(msg);
    }

    void
    sensor_callback(
        const std_msgs::msg::Int16MultiArray::SharedPtr msg) {

        ctrlPkt_.updateRx(msg->data);

        // double position_deg = ctrlPkt_.getPosition(TARGET_MOTOR);
        // double velocity_erpm = ctrlPkt_.getVelocity(TARGET_MOTOR);
        // double current_a = ctrlPkt_.getCurrent(TARGET_MOTOR);
        // int temperature_c = ctrlPkt_.getTemperature(TARGET_MOTOR);
        // int16_t error = ctrlPkt_.getError(TARGET_MOTOR);

        // 以降、受信データを使った処理を記述

        // RCLCPP_INFO_THROTTLE(
        //     get_logger(), *get_clock(), 500,
        //     "motor%d pos=%.1fdeg speed=%.0fERPM current=%.2fA temp=%ddegC error=%s",
        //     TARGET_MOTOR, position_deg, velocity_erpm, current_a, temperature_c,
        //     Ros2CanCubemarsPacketController::errorText(error));

        // 受信データ処理ここまで
    }

    // 全モータをゼロ速度指令(その場停止)にして送信する
    void send_zero_and_stop() {
        ctrlPkt_.stopAll();
        publisher_timer_callback();
    }

    uint8_t tx_device_id_;
    uint8_t rx_device_id_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    Ros2CanCubemarsPacketController ctrlPkt_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet Ros2Can CubeMars";
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

    auto ros2can_cubemars = std::make_shared<Ros2CanCubemarsControl>(TX_DEVICE_ID, RX_DEVICE_ID);
    exec.add_node(ros2can_cubemars);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
