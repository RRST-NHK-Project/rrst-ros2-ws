/*
ros2can (MODE_CUBEMARS) ノードのホスト側サンプルプログラム。
CubeMars AKシリーズ(Servo(CAN)モード)のモータ1(CAN_ID=101)を、setPosition()で
0~90degの間を正弦波で往復させ、位置・速度・電流・温度・エラーの帰還をログ出力する。
Ros2CanCubemarsPacketControllerを使用してスロット割当を意識せずに送受信配列へ
アクセスする(ジョイスティック等の外部入力は不要、実行するとその場で動き出す)。

joyスティックでMIT(Force Control)モードを使った実例は cr26_soki/src/ros2can_practice.cpp
を参照。
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
// ================================================================================

#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>

#include "common/Ros2CanCubemarsPacketController.hpp"

// 以下マイコンに合わせて設定
// TX/RX_DEVICE_ID は CubeMarsドライバマイコン自身のシリアルフレームDEVICE_ID
// (firmware/xiao-esp32-s3_can2io/src/config.hpp の DEVICE_ID)。
// CANバス上の各モータのCAN_ID (101,102,103,104、config.hppのCUBEMARS_MOTOR_ID_n)
// とは別物なので注意。
#define TX_DEVICE_ID 1
#define RX_DEVICE_ID 1

#define PUBLISH_RATE_MS 20  // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

// 制御対象のモータ (0-origin: 0~3、CAN_ID 101~104に対応)
#define TARGET_MOTOR 0

#define SWEEP_PERIOD_SEC 4.0
#define SWEEP_CENTER_DEG 45.0
#define SWEEP_AMPLITUDE_DEG 45.0  // 0~90degの間で往復

class CubemarsPositionExample : public rclcpp::Node {
public:
    CubemarsPositionExample(uint8_t tx_device_id, uint8_t rx_device_id)
        : Node("ros2can_example_cubemars_position"),
          tx_device_id_(tx_device_id),
          rx_device_id_(rx_device_id) {

        publisher_ = create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(tx_device_id_), 10);

        sensor_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(rx_device_id_), 10,
            std::bind(&CubemarsPositionExample::sensor_callback, this, std::placeholders::_1));

        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&CubemarsPositionExample::timer_callback, this));

        start_time_ = now();

        // firmware側にCAN/シリアル途絶時のフェイルセーフは無く、最後に受信した指令を
        // 保持し続ける。ノード終了時は全モータをゼロ速度に戻す(唯一の安全装置)。
        rclcpp::on_shutdown([this]() { send_zero_and_stop(); });

        RCLCPP_INFO(get_logger(), "serial_tx_%d started (motor%d position sweep).",
                    tx_device_id_, TARGET_MOTOR + 1);
    }

private:
    void timer_callback() {
        double t = (now() - start_time_).seconds();
        double target_deg = SWEEP_CENTER_DEG +
            SWEEP_AMPLITUDE_DEG * std::sin(2.0 * M_PI * t / SWEEP_PERIOD_SEC);
        ctrlPkt_.setPosition(TARGET_MOTOR, target_deg);

        std_msgs::msg::Int16MultiArray msg;
        msg.data = ctrlPkt_.toVector();
        publisher_->publish(msg);
    }

    void sensor_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        ctrlPkt_.updateRx(msg->data);

        double position_deg = ctrlPkt_.getPosition(TARGET_MOTOR);
        double velocity_erpm = ctrlPkt_.getVelocity(TARGET_MOTOR);
        double current_a = ctrlPkt_.getCurrent(TARGET_MOTOR);
        int temperature_c = ctrlPkt_.getTemperature(TARGET_MOTOR);
        int16_t error = ctrlPkt_.getError(TARGET_MOTOR);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "motor%d pos=%.1fdeg speed=%.0fERPM current=%.2fA temp=%ddegC error=%s",
            TARGET_MOTOR + 1, position_deg, velocity_erpm, current_a, temperature_c,
            Ros2CanCubemarsPacketController::errorText(error));
    }

    // 全モータをゼロ速度指令(その場停止)にして送信する
    void send_zero_and_stop() {
        ctrlPkt_.stopAll();
        std_msgs::msg::Int16MultiArray msg;
        msg.data = ctrlPkt_.toVector();
        publisher_->publish(msg);
    }

    uint8_t tx_device_id_;
    uint8_t rx_device_id_;

    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;

    Ros2CanCubemarsPacketController ctrlPkt_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CubemarsPositionExample>(TX_DEVICE_ID, RX_DEVICE_ID);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
