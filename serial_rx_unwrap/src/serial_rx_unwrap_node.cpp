/*
ros2can の serial_rx_[DEVICE_ID] (Int16MultiArray, 24要素、各スロットint16で
±32768折り返し)を購読し、全スロットをCounterUnwrapperで連続値化して
serial_rx_[DEVICE_ID]_unwrapped (Int32MultiArray) として配信するブリッジノード。

ros2can本体は変更しない(トピック契約: serial_rx_[ID] は従来通り)。
unwrap処理は独立したこのノードが担い、ros2canのプロセス/スレッド事情
(GUIスレッドの詰まり等)に依存しない別プロセスとして動作する。

1ノード = 1 device_id を担当する。複数デバイスを扱う場合は launch で
パラメータを変えて複数インスタンス起動する。
*/

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "serial_rx_unwrap/counter_unwrapper.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

namespace {
constexpr int kSlotCount = 24;
constexpr int64_t kCountsPerWrap = 1LL << 16;
}  // namespace

class SerialRxUnwrapNode : public rclcpp::Node {
public:
    SerialRxUnwrapNode() : Node("serial_rx_unwrap_node") {
        device_id_ = declare_parameter<int>("device_id", 1);

        unwrappers_.assign(kSlotCount, serial_rx_unwrap::CounterUnwrapper(kCountsPerWrap));

        pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
            "/serial_rx_" + std::to_string(device_id_) + "_unwrapped", 10);
        sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
            "/serial_rx_" + std::to_string(device_id_), 10,
            std::bind(&SerialRxUnwrapNode::on_rx, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "serial_rx_unwrap_node started: device_id=%d (%s -> %s)",
                    device_id_,
                    ("/serial_rx_" + std::to_string(device_id_)).c_str(),
                    ("/serial_rx_" + std::to_string(device_id_) + "_unwrapped").c_str());
    }

private:
    void on_rx(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        const int count = std::min<int>(kSlotCount, static_cast<int>(msg->data.size()));

        std_msgs::msg::Int32MultiArray out;
        out.data.assign(kSlotCount, 0);
        for (int i = 0; i < count; ++i) {
            out.data[i] = static_cast<int32_t>(unwrappers_[i].update(msg->data[i]));
        }
        pub_->publish(out);
    }

    int device_id_ = 1;
    std::vector<serial_rx_unwrap::CounterUnwrapper> unwrappers_;

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialRxUnwrapNode>());
    rclcpp::shutdown();
    return 0;
}
