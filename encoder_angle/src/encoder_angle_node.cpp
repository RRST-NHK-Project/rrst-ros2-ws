/*
ros2can の serial_rx_[DEVICE_ID] (Int16MultiArray, 24要素) から汎用IOノード
(xiao-esp32-s3_can2io, MODE_IO/MODE_CAN)のロータリーエンコーダ入力
ENC1/ENC2 スロットだけを取り出し、CounterUnwrapper で連続値化した上で
連続角度 [deg] として配信するノード。

[スロット位置]
README.md記載の対応スロットマッピングの通り、1ノードあたり
[SW1, SW2, SW3, ENC1, ENC2] の5スロット(既定 slots_per_node=5)が並び、
グローバルスロット index = node_index(0-origin) * slots_per_node + local_offset
で求まる (ENC1: local_offset=3, ENC2: local_offset=4 が既定)。

[連続角度への変換]
ENC1/ENC2 の生値は xiao-esp32-s3_can2io 側の PCNT (パルスカウンタ) が
出力する int16 の生カウントで、+方向は32767到達、-方向は-32768到達で
それぞれ独立に0へリセットされる(実機で確認済み。詳細は
counter_unwrapper.hpp の冒頭コメント参照)。これを
CounterUnwrapper で連続値(ticks)に復元し、
angle_deg = ticks * (360 / encoder_ppr) * sign
で連続角度に変換する。encoder_ppr は1回転あたりの生カウント数
(x4逓倍後。defs.hpp の PPR = ENC_PPR_SPEC*4 に対応、既定8192)。

1ノード(=物理的に2本のロータリーエンコーダが生えたIOノード1台)につき
本ノード1インスタンスを起動する。複数ノードのエンコーダを使う場合は
launch でパラメータを変えて複数インスタンス起動する。
*/

#include <cstdint>
#include <string>

#include "encoder_angle/counter_unwrapper.hpp"
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int64.hpp>

namespace {
// serial_bridge/ros2can プロトコルの固定スロット数
// [ros2can/ros2can/frame_codec.py の SLOT_COUNT と一致させること]
constexpr int kSlotCount = 24;
// PCNT の生カウントは、+方向は32767到達、-方向は-32768到達でそれぞれ
// 独立に0へリセットされる(2の補数オーバーフローではない。実機確認済み。
// [counter_unwrapper.hpp 冒頭コメント参照])。リセット幅は正負とも32768。
constexpr int64_t kCountsPerWrap = 1LL << 15;
}  // namespace

class EncoderAngleNode : public rclcpp::Node {
public:
    EncoderAngleNode() : Node("encoder_angle_node") {
        device_id_ = declare_parameter<int>("device_id", 101);
        node_index_ = declare_parameter<int>("node_index", 0);
        slots_per_node_ = declare_parameter<int>("slots_per_node", 5);
        const int enc1_slot_offset = declare_parameter<int>("enc1_slot_offset", 3);
        const int enc2_slot_offset = declare_parameter<int>("enc2_slot_offset", 4);

        // 1回転あたりの生カウント数(x4逓倍後)。xiao-esp32-s3_can2io の既定値
        // (ENC_PPR_SPEC=2048 * 4) に合わせて8192を既定にしている。
        encoder_ppr_ = declare_parameter<double>("encoder_ppr", 8192.0);
        enc1_sign_ = declare_parameter<double>("enc1_sign", 1.0);
        enc2_sign_ = declare_parameter<double>("enc2_sign", 1.0);

        const double ambiguous_margin_ratio =
            declare_parameter<double>("ambiguous_margin_ratio", 0.1);
        const int trend_noise_floor = declare_parameter<int>("trend_noise_floor", 4);

        enc1_slot_ = node_index_ * slots_per_node_ + enc1_slot_offset;
        enc2_slot_ = node_index_ * slots_per_node_ + enc2_slot_offset;
        deg_per_count_ = (encoder_ppr_ > 0.0) ? (360.0 / encoder_ppr_) : 0.0;

        if (enc1_slot_ < 0 || enc1_slot_ >= kSlotCount || enc2_slot_ < 0 ||
            enc2_slot_ >= kSlotCount) {
            throw std::runtime_error(
                "ENC1/ENC2 slot index is out of range (check node_index/slots_per_node/"
                "enc1_slot_offset/enc2_slot_offset)");
        }
        if (encoder_ppr_ <= 0.0) {
            throw std::runtime_error("encoder_ppr must be > 0");
        }

        unwrapper1_ = encoder_angle::CounterUnwrapper(kCountsPerWrap, ambiguous_margin_ratio,
                                                        trend_noise_floor);
        unwrapper2_ = encoder_angle::CounterUnwrapper(kCountsPerWrap, ambiguous_margin_ratio,
                                                        trend_noise_floor);

        // serial_rx_[ID] は ros2can 側がルート名前空間で作るトピックなので、
        // 本ノードを名前空間付きで複数起動しても解決先がずれないよう絶対名にする。
        // (出力トピックは逆にノード毎に分離したいので相対名のままにする)
        rx_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
            "/serial_rx_" + std::to_string(device_id_), 10,
            std::bind(&EncoderAngleNode::on_rx, this, std::placeholders::_1));

        enc1_angle_pub_ = create_publisher<std_msgs::msg::Float64>("encoder1/angle_deg", 10);
        enc1_ticks_pub_ = create_publisher<std_msgs::msg::Int64>("encoder1/ticks", 10);
        enc2_angle_pub_ = create_publisher<std_msgs::msg::Float64>("encoder2/angle_deg", 10);
        enc2_ticks_pub_ = create_publisher<std_msgs::msg::Int64>("encoder2/ticks", 10);

        RCLCPP_INFO(get_logger(),
                    "encoder_angle_node started: device_id=%d node_index=%d "
                    "enc1_slot=%d enc2_slot=%d encoder_ppr=%.2f (%.6f deg/count) "
                    "sign=(%.0f, %.0f)",
                    device_id_, node_index_, enc1_slot_, enc2_slot_, encoder_ppr_,
                    deg_per_count_, enc1_sign_, enc2_sign_);
    }

private:
    void on_rx(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        const int count = static_cast<int>(msg->data.size());
        if (count <= enc1_slot_ || count <= enc2_slot_) {
            return;
        }

        const int64_t ticks1 = unwrapper1_.update(msg->data[enc1_slot_]);
        const int64_t ticks2 = unwrapper2_.update(msg->data[enc2_slot_]);

        publish_encoder(ticks1, enc1_sign_, enc1_angle_pub_, enc1_ticks_pub_);
        publish_encoder(ticks2, enc2_sign_, enc2_angle_pub_, enc2_ticks_pub_);
    }

    void publish_encoder(int64_t ticks, double sign,
                          const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr &angle_pub,
                          const rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr &ticks_pub) {
        std_msgs::msg::Float64 angle_msg;
        angle_msg.data = static_cast<double>(ticks) * deg_per_count_ * sign;
        angle_pub->publish(angle_msg);

        std_msgs::msg::Int64 ticks_msg;
        ticks_msg.data = ticks;
        ticks_pub->publish(ticks_msg);
    }

    int device_id_ = 101;
    int node_index_ = 0;
    int slots_per_node_ = 5;
    int enc1_slot_ = 3;
    int enc2_slot_ = 4;
    double encoder_ppr_ = 8192.0;
    double deg_per_count_ = 360.0 / 8192.0;
    double enc1_sign_ = 1.0;
    double enc2_sign_ = 1.0;

    encoder_angle::CounterUnwrapper unwrapper1_;
    encoder_angle::CounterUnwrapper unwrapper2_;

    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr rx_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr enc1_angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr enc1_ticks_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr enc2_angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr enc2_ticks_pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EncoderAngleNode>());
    rclcpp::shutdown();
    return 0;
}
