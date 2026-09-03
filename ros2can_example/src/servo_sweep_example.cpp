/*
ros2can (MODE_CAN_HOST, 既定プロファイル xiao_smd_can_host) ノードの
ホスト側最小サンプルプログラム。
ノード1(CAN_ID=101)のSERVO1を正弦波で往復させ、SW1とENC1の帰還をログ出力する。
Ros2CanPacketControllerを使用してスロット割当を意識せずに送受信配列へアクセスする。
*/

// ===== Ros2CanPacketController (common/Ros2CanPacketController.hpp) 関数一覧 =====
// void setServo(int node, int servo_no, int deg)
//   指定ノード(node: 0-origin)のSERVOn(servo_no: 1-3)を角度[deg]で設定する
//   (範囲外degはクランプ、node/servo_no範囲外は無視)。
//
// bool getSW(int node, int sw_no) const
//   指定ノードのSWn(1-3)状態を取得する(SERVOnとピン共有、MULTIn=0のときのみ有効)。
//
// int16_t getEnc(int node, int enc_no) const
//   指定ノードのENCn(1-2)カウンタ値を取得する。
//
// static int canId(int node)
//   node(0-origin)に対応するCAN_IDの既定値(101,102,103,104)を返す。
//
// void updateRx(const std::vector<int16_t> &data)
//   受信したInt16MultiArray相当のdataでrx_配列を更新する(sensor_callback等で使用)。
//
// std::vector<int16_t> toVector() const
//   送信用配列tx_をstd::vectorに変換して取得する(publish直前に使用)。
//
// int16_t &operator[](int index) / const int16_t &operator[](int index) const
//   tx_配列への直接アクセス(グローバルslot index指定、通常はsetServo等を優先)。
// ================================================================================

#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>

#include "common/Ros2CanPacketController.hpp"

// 以下マイコンに合わせて設定
// TX/RX_DEVICE_ID は CANホストマイコン自身のシリアルフレームDEVICE_ID
// (firmware/xiao-esp32-s3_can2io/src/config.hpp の DEVICE_ID)。
#define TX_DEVICE_ID 1
#define RX_DEVICE_ID 1

#define PUBLISH_RATE_MS 50  // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

// 制御対象のノード (0-origin: 0~3、CAN_ID 101~104に対応)
#define TARGET_NODE 0

#define SWEEP_PERIOD_SEC 4.0  // 往復1周期の時間[秒]

class ServoSweepExample : public rclcpp::Node {
public:
    ServoSweepExample(uint8_t tx_device_id, uint8_t rx_device_id)
        : Node("ros2can_example_servo_sweep"),
          tx_device_id_(tx_device_id),
          rx_device_id_(rx_device_id) {

        /*
        既定プロファイル(xiao_smd_can_host)は24スロットを4ノード x 5スロットに分配する。
        ノード1(0-origin: node=0)の担当スロットは以下の通り(local index 0-4):
          指令(TX): SERVO1, SERVO2, SERVO3, (予備), (予備)
          帰還(RX): SW1,    SW2,    SW3,    ENC1,    ENC2
        SERVOn と SWn はピン共有(ファームウェア config.hpp の MULTIn で切替、
        0=スイッチ入力/1=サーボ出力)。setServo()/getSW()/getEnc()がノード・
        ローカル番号からグローバルスロットへの変換を代行するので、通常は
        この割当を直接意識する必要はない。
        */

        publisher_ = create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(tx_device_id_), 10);

        sensor_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(rx_device_id_), 10,
            std::bind(&ServoSweepExample::sensor_callback, this, std::placeholders::_1));

        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&ServoSweepExample::timer_callback, this));

        start_time_ = now();

        // firmware側にCAN/シリアル途絶時のフェイルセーフは無く、最後に受信した指令を
        // 保持し続ける。ノード終了時はSERVOを0degへ戻す(唯一の安全装置)。
        rclcpp::on_shutdown([this]() { send_zero_and_stop(); });

        RCLCPP_INFO(get_logger(), "serial_tx_%d started (node%d SERVO1 sweep).",
                    tx_device_id_, TARGET_NODE + 1);
    }

private:
    void timer_callback() {
        double t = (now() - start_time_).seconds();
        // 0~270degの範囲で正弦波往復
        int deg = 135 + static_cast<int>(135.0 * std::sin(2.0 * M_PI * t / SWEEP_PERIOD_SEC));
        ctrlPkt_.setServo(TARGET_NODE, /*servo_no=*/1, deg);

        std_msgs::msg::Int16MultiArray msg;
        msg.data = ctrlPkt_.toVector();
        publisher_->publish(msg);
    }

    void sensor_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        ctrlPkt_.updateRx(msg->data);

        bool sw1 = ctrlPkt_.getSW(TARGET_NODE, 1);
        int16_t enc1 = ctrlPkt_.getEnc(TARGET_NODE, 1);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "node%d SW1=%s ENC1=%d", TARGET_NODE + 1, sw1 ? "ON" : "OFF", enc1);
    }

    // SERVO1を0degに戻して送信する
    void send_zero_and_stop() {
        ctrlPkt_.setServo(TARGET_NODE, 1, 0);
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

    Ros2CanPacketController ctrlPkt_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ServoSweepExample>(TX_DEVICE_ID, RX_DEVICE_ID);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
