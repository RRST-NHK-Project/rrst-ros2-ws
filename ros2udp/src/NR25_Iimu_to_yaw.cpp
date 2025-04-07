//YAW角取得成功

// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/imu.hpp"
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>

// class ImuYawNode : public rclcpp::Node
// {
// public:
//     ImuYawNode() : Node("imu_yaw_node")
//     {
//         imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
//             "/imu", rclcpp::SensorDataQoS(),
//             std::bind(&ImuYawNode::imu_callback, this, std::placeholders::_1));
//     }

// private:
//     void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
//     {
//         // 四元数を取得
//         tf2::Quaternion q(
//             msg->orientation.x,
//             msg->orientation.y,
//             msg->orientation.z,
//             msg->orientation.w);

//         // オイラー角 (roll, pitch, yaw) に変換
//         double roll, pitch, yaw;
//         tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

//         // YAW角度（ラジアン → 度）を表示
//         RCLCPP_INFO(this->get_logger(), "YAW: %.2f [deg]", yaw * 180.0 / M_PI);
//     }

//     rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ImuYawNode>());
//     rclcpp::shutdown();
//     return 0;
// }

//ここまで

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class ImuYawToRvizNode : public rclcpp::Node
{
public:
    ImuYawToRvizNode() : Node("imu_yaw_to_rviz")
    {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", rclcpp::SensorDataQoS(),
            std::bind(&ImuYawToRvizNode::imu_callback, this, std::placeholders::_1));

        // Marker パブリッシャー
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/yaw_marker", 10);
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 四元数を取得
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        // オイラー角 (roll, pitch, yaw) に変換
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // YAW角度（ラジアン → 度）を表示
        RCLCPP_INFO(this->get_logger(), "YAW: %.2f [deg]", yaw * 180.0 / M_PI);

        // マーカーを作成してYAW角度を反映
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";  // RVizでのフレーム
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "imu_yaw";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = sin(yaw / 2);  // 四元数形式でYAW角度を設定
        marker.pose.orientation.w = cos(yaw / 2);
        marker.scale.x = 0.5;  // 矢印の太さ
        marker.scale.y = 0.1;  // 矢印の幅
        marker.scale.z = 0.1;  // 矢印の高さ
        marker.color.a = 1.0;  // 透明度
        marker.color.r = 1.0;  // 赤色
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        // Markerをパブリッシュ
        marker_pub_->publish(marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuYawToRvizNode>());
    rclcpp::shutdown();
    return 0;
}

