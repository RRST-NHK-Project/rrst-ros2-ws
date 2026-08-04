/*
位置指令をPID制御で速度指令に変換しros2can経由でCANノードへ送信するノード。

位置指令(std_msgs/Float64, deg)を common/PID.hpp の PIDController で位置PID制御
し、速度指令(rpm)に変換して ros2can 互換の serial_tx_[DEVICE_ID] トピック
(Int16MultiArray, 24要素)経由で CANホストマイコン配下の FOC モータノード
(b-g431-esc1_can2io, SimpleFOC)へ送信する。

速度制御そのものはマイコン側(内蔵の速度PID)が行うため、本ノードが担うのは
位置ループ(外側)のみ。b-g431-esc1_can2io は CAN 途絶時のフェイルセーフを持たず
最後に受信した target_velocity を保持し続けるため、位置指令が
command_timeout_sec 以上途絶えた場合は本ノード側で速度指令を0にフォールバック
する(唯一の安全装置)。

位置フィードバックの取得元(feedback_device_id/feedback_node_index)は、速度指令の
送信先(device_id/node_index)とは独立して設定できる。ESC自身のエンコーダではなく
CANホスト機側に接続されたエンコーダ(汎用IOノードのENC1/ENC2等)を使う構成を
既定として想定しており、パラメータで任意の組み合わせに変更できる。
feedback スロットの値は encoder_ppr を 0 より大きくすると「1周あたりのパルス数」
として扱い(360/encoder_ppr [deg/count]でスケール)、0のままなら legacy な
0.1deg/LSB スケール(angle_scale_deg)を使う。

生のスロットは int16(±32768)でラップするが、そのラップ検出は本ノードでは行わず
ros2can 側(HardwareManager がシリアルフレームを受信するループ内、サンプルが密な
場所)で行い、既に連続値化された serial_rx_[ID]_unwrapped トピック(Int32MultiArray)
を購読する。ROSトピック配信がGUIスレッドの詰まり等で疎になっても、ros2can側で
既に連続値化済みのため、本ノード側でラップを誤判定することはない
(過去に本ノード側でも同様のunwrapを行っていたが、疎なサンプルに対する
「絶対値最小」ヒューリスティックは半周期に近い欠落で原理的に誤判定しうるため、
サンプルが密なros2can側に移設した)。

1ノード = 1軸を担当する。複数軸を制御する場合は launch でパラメータを変えて
複数インスタンス起動する。
*/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "common/PID.hpp"
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

namespace {

// serial_bridge/ros2can プロトコルの固定スロット数
// [ros2can/ros2can/frame_codec.py の SLOT_COUNT と一致させること]
constexpr int kSlotCount = 24;

int16_t clamp_i16(double value) {
    const double clamped = std::clamp(value, -32768.0, 32767.0);
    return static_cast<int16_t>(std::lround(clamped));
}

}  // namespace

class EscCtrlNode : public rclcpp::Node {
public:
    EscCtrlNode() : Node("esc_ctrl_node") {
        // --- 速度指令の送信先 (ESC/FOCノード側) ---
        device_id_ = declare_parameter<int>("device_id", 1);
        node_index_ = declare_parameter<int>("node_index", 0);
        slots_per_node_ = declare_parameter<int>("slots_per_node", 5);
        const int target_velocity_slot_offset =
            declare_parameter<int>("target_velocity_slot_offset", 0);
        velocity_limit_rpm_ = std::abs(declare_parameter<double>("velocity_limit_rpm", 450.0));

        // --- 位置フィードバックの取得元 (既定はホスト機側エンコーダを想定) ---
        // device_id/node_index とは独立に指定できる(同一ホストの別ノードでも、
        // 全く別の device_id でもよい)。未指定時は速度指令の送信先と同じ値になる。
        feedback_device_id_ = declare_parameter<int>("feedback_device_id", device_id_);
        feedback_node_index_ = declare_parameter<int>("feedback_node_index", node_index_);
        feedback_slots_per_node_ =
            declare_parameter<int>("feedback_slots_per_node", slots_per_node_);
        const int feedback_slot_offset =
            declare_parameter<int>("feedback_slot_offset", 0);
        // encoder_ppr > 0 : feedbackスロットを「1周あたりパルス数」の生カウントとして
        //   扱い、360/encoder_ppr [deg/count] でスケールする(ホスト機ENC1/ENC2等、
        //   汎用IOノードのロータリーエンコーダ入力を想定)。
        // encoder_ppr <= 0: legacy な 0.1deg/LSB スケール(angle_scale_deg)を使う
        //   (FOCノード自身の角度フィードバック等、スケール済みの値を想定)。
        encoder_ppr_ = declare_parameter<double>("encoder_ppr", 0.0);
        angle_scale_deg_ = declare_parameter<double>("angle_scale_deg", 0.1);
        // モーターの正転方向(target_velocityが正の時に実際に回る向き)と、
        // フィードバックに使うエンコーダのカウント増加方向が逆な場合に-1.0にする。
        // (別体のエンコーダ等、ESC自身の内部基準と一致している保証が無いため)
        feedback_sign_ = declare_parameter<double>("feedback_sign", 1.0);

        const double kp = declare_parameter<double>("kp", 1.0);
        const double ki = declare_parameter<double>("ki", 0.0);
        const double kd = declare_parameter<double>("kd", 0.0);
        const double control_rate_hz = declare_parameter<double>("control_rate_hz", 50.0);
        const double command_timeout_sec = declare_parameter<double>("command_timeout_sec", 0.5);
        show_info_logs_ = declare_parameter<bool>("show_info_logs", true);

        target_velocity_slot_ = node_index_ * slots_per_node_ + target_velocity_slot_offset;
        feedback_slot_ = feedback_node_index_ * feedback_slots_per_node_ + feedback_slot_offset;
        feedback_deg_per_count_ = (encoder_ppr_ > 0.0) ? (360.0 / encoder_ppr_) : angle_scale_deg_;

        if (target_velocity_slot_ < 0 || target_velocity_slot_ >= kSlotCount) {
            throw std::runtime_error(
                "target_velocity slot index is out of range (check node_index/"
                "slots_per_node/target_velocity_slot_offset)");
        }
        if (feedback_slot_ < 0 || feedback_slot_ >= kSlotCount) {
            throw std::runtime_error(
                "feedback slot index is out of range (check feedback_node_index/"
                "feedback_slots_per_node/feedback_slot_offset)");
        }

        command_timeout_ = rclcpp::Duration::from_seconds(command_timeout_sec);
        control_period_sec_ = 1.0 / control_rate_hz;

        pid_ = std::make_unique<PIDController>(
            static_cast<float>(kp), static_cast<float>(ki), static_cast<float>(kd),
            static_cast<float>(velocity_limit_rpm_));

        tx_data_.assign(kSlotCount, 0);

        // serial_tx_/serial_rx_ は ros2can 側がルート名前空間で作るトピックなので、
        // 本ノードを名前空間付きで複数起動しても解決先がずれないよう絶対名にする。
        // (position_command は逆にノード毎に分離したいので相対名のままにする)
        tx_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>(
            "/serial_tx_" + std::to_string(device_id_), 10);
        // ros2can が受信ループ内で既に連続値化(unwrap)して配信するトピックを使う
        // (このノード自身ではラップ検出を行わない。詳細はファイル冒頭のコメント参照)
        rx_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            "/serial_rx_" + std::to_string(feedback_device_id_) + "_unwrapped", 10,
            std::bind(&EscCtrlNode::on_rx, this, std::placeholders::_1));
        position_command_sub_ = create_subscription<std_msgs::msg::Float64>(
            "position_command", 10,
            std::bind(&EscCtrlNode::on_position_command, this, std::placeholders::_1));

        timer_ = create_wall_timer(
            std::chrono::duration<double>(control_period_sec_),
            std::bind(&EscCtrlNode::on_control_timer, this));

        rclcpp::on_shutdown([this]() { send_zero_and_stop(); });

        RCLCPP_INFO(get_logger(),
                    "esc_ctrl_node started: device_id=%d node_index=%d target_velocity_slot=%d | "
                    "feedback_device_id=%d feedback_node_index=%d feedback_slot=%d "
                    "(%s, %.5f deg/count, sign=%.0f) | rate=%.1fHz",
                    device_id_, node_index_, target_velocity_slot_,
                    feedback_device_id_, feedback_node_index_, feedback_slot_,
                    (encoder_ppr_ > 0.0) ? "encoder_ppr" : "angle_scale_deg",
                    feedback_deg_per_count_, feedback_sign_, control_rate_hz);
    }

private:
    void on_position_command(const std_msgs::msg::Float64::SharedPtr msg) {
        target_position_deg_ = msg->data;
        target_stamp_ = now();
        has_target_ = true;
    }

    void on_rx(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (static_cast<int>(msg->data.size()) <= feedback_slot_) {
            return;
        }
        const int64_t unwrapped = msg->data[feedback_slot_];
        measured_position_deg_ =
            static_cast<double>(unwrapped) * feedback_deg_per_count_ * feedback_sign_;
        has_feedback_ = true;
    }

    void on_control_timer() {
        const bool stale = !has_target_ || (now() - target_stamp_) > command_timeout_;
        double velocity_cmd = 0.0;

        if (stale || !has_feedback_) {
            pid_->reset();
            was_stale_ = true;
        } else {
            pid_->set_target(static_cast<float>(target_position_deg_));
            if (was_stale_) {
                // last_current_ をprimeして、途絶中の変化による微分の跳ねを防ぐ
                pid_->update(static_cast<float>(measured_position_deg_), 1.0f);
                was_stale_ = false;
            }
            velocity_cmd = pid_->update(static_cast<float>(measured_position_deg_),
                                         static_cast<float>(control_period_sec_));
        }

        tx_data_[target_velocity_slot_] = clamp_i16(velocity_cmd);
        publish_tx();

        if (show_info_logs_) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 500,
                "target=%.2fdeg measured=%.2fdeg err=%.2fdeg cmd=%.1frpm stale=%d "
                "has_feedback=%d",
                target_position_deg_, measured_position_deg_,
                target_position_deg_ - measured_position_deg_, velocity_cmd, stale,
                has_feedback_);
        }
    }

    void publish_tx() {
        std_msgs::msg::Int16MultiArray msg;
        msg.data = tx_data_;
        tx_pub_->publish(msg);
    }

    void send_zero_and_stop() {
        tx_data_[target_velocity_slot_] = 0;
        publish_tx();
    }

    int device_id_ = 1;
    int node_index_ = 0;
    int slots_per_node_ = 5;
    int target_velocity_slot_ = 0;

    int feedback_device_id_ = 1;
    int feedback_node_index_ = 0;
    int feedback_slots_per_node_ = 5;
    int feedback_slot_ = 0;
    double encoder_ppr_ = 0.0;
    double angle_scale_deg_ = 0.1;
    double feedback_deg_per_count_ = 0.1;
    double feedback_sign_ = 1.0;

    double velocity_limit_rpm_ = 450.0;
    double control_period_sec_ = 0.02;
    rclcpp::Duration command_timeout_{0, 0};
    bool show_info_logs_ = true;

    std::unique_ptr<PIDController> pid_;

    std::vector<int16_t> tx_data_;
    bool has_target_ = false;
    bool has_feedback_ = false;
    bool was_stale_ = true;
    double target_position_deg_ = 0.0;
    double measured_position_deg_ = 0.0;
    rclcpp::Time target_stamp_;

    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr tx_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr rx_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_command_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EscCtrlNode>());
    rclcpp::shutdown();
    return 0;
}
