/*
RRST-NHK-Project 2025
LD19のスキャンデータをフィルタリングするノード
*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath>
#include <limits>
#include <unistd.h>

float passed_range = 30.0; // ±30度の範囲を指定

class LD19FrontScanNode : public rclcpp::Node {
public:
    LD19FrontScanNode() : Node("LD19_FrontScan_Node") {
        publisher_filtered_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>("filtered_scan", 10);
        publisher_nearest_point_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("nearest_point", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/ldlidar_node/scan", 10, std::bind(&LD19FrontScanNode::scan_callback, this, std::placeholders::_1));

        nearest_point_msg_.data.resize(1, 0.0);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::vector<float>::size_type center_index = msg->ranges.size() / 2;
        float min_distance = std::numeric_limits<float>::infinity();

        // ±30度に相当するインデックス範囲を計算
        float angle_width_rad = passed_range * M_PI / 180.0;
        std::vector<float>::size_type angle_range = static_cast<std::vector<float>::size_type>(
            angle_width_rad / msg->angle_increment);

        std::vector<float>::size_type start = std::max(center_index > angle_range ? center_index - angle_range : 0, size_t(0));
        std::vector<float>::size_type end = std::min(center_index + angle_range, msg->ranges.size() - 1);

        // 最小距離を探索
        for (std::vector<float>::size_type i = start; i <= end; ++i) {
            if (i < msg->ranges.size() && std::isfinite(msg->ranges[i])) {
                float distance = msg->ranges[i];
                if (distance < min_distance) {
                    min_distance = distance;
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "前方最近傍点距離: %.2f m", min_distance);

        // フィルタリングされたLaserScanメッセージを作成
        sensor_msgs::msg::LaserScan filtered_scan_msg = *msg;

        // 指定範囲以外のデータを無限大に設定
        for (std::vector<float>::size_type i = 0; i < msg->ranges.size(); ++i) {
            if (i < start || i > end) {
                filtered_scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
            }
        }

        // フィルタリングされたLaserScanメッセージをパブリッシュ
        publisher_filtered_scan_->publish(filtered_scan_msg);

        // 最近傍点距離をパブリッシュ
        nearest_point_msg_.data[0] = min_distance;
        publisher_nearest_point_->publish(nearest_point_msg_);
    }

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_filtered_scan_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_nearest_point_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    std_msgs::msg::Float64MultiArray nearest_point_msg_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet LD19 FrontScan";
    int result = std::system(figletout.c_str());
    if (result != 0) {
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        std::cerr << "Please install 'figlet' with the following command:" << std::endl;
        std::cerr << "sudo apt install figlet" << std::endl;
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    }

    rclcpp::spin(std::make_shared<LD19FrontScanNode>());
    rclcpp::shutdown();
    return 0;
}
