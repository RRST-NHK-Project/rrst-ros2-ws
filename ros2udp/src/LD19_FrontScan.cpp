/*
RRST-NHK-Project 2025
LD19のスキャンデータをフィルタリングするノード
*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <limits>

float passed_range = 30.0; // ±30度の範囲を指定

class LD19FrontScanNode : public rclcpp::Node {
public:
    LD19FrontScanNode() : Node("LD19_FrontScan_Node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("filtered_scan", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/ldlidar_node/scan", 10, std::bind(&LD19FrontScanNode::scan_callback, this, std::placeholders::_1));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::vector<float>::size_type center_index = msg->ranges.size() / 2; // インデックスの総数を2で割ることで中央のインデックスを取得
        float min_distance = std::numeric_limits<float>::infinity();         // 初期は最大値

        // ±10度範囲の最近傍距離を探す
        std::vector<float>::size_type angle_range = static_cast<std::vector<float>::size_type>(
            passed_range * M_PI / 180.0 / msg->angle_increment); // ±30度
        std::vector<float>::size_type start = center_index - angle_range;
        std::vector<float>::size_type end = center_index + angle_range;

        // 最小距離を探す
        for (std::vector<float>::size_type i = start; i <= end; ++i) {
            if (i < msg->ranges.size() && std::isfinite(msg->ranges[i])) {
                float distance = msg->ranges[i];
                if (distance < min_distance) {
                    min_distance = distance;
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "前方最近距離: %.2f m", min_distance);

        // フィルタリングされたLaserScanメッセージを作成
        sensor_msgs::msg::LaserScan filtered_scan_msg = *msg; // 元のメッセージをコピー

        // ±10度範囲内のデータだけを保持
        for (std::vector<float>::size_type i = 0; i < msg->ranges.size(); ++i) {
            if (i < start || i > end) {
                filtered_scan_msg.ranges[i] = std::numeric_limits<float>::infinity(); // 範囲外のデータを無限大に設定
            }
        }

        // RVizに表示されるために必要なパラメータを設定
        filtered_scan_msg.angle_min = msg->angle_min;             // 元のangle_minを設定
        filtered_scan_msg.angle_max = msg->angle_max;             // 元のangle_maxを設定
        filtered_scan_msg.angle_increment = msg->angle_increment; // 元のangle_incrementを設定
        filtered_scan_msg.range_min = msg->range_min;             // 元のrange_minを設定
        filtered_scan_msg.range_max = msg->range_max;             // 元のrange_maxを設定

        // フィルタリングされたLaserScanをpublish
        publisher_->publish(filtered_scan_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LD19FrontScanNode>());
    rclcpp::shutdown();
    return 0;
}
