// NHK2026ロボコン
// 経路->ROS2のパブリッシャー
// 2026/03/12(作業中)

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"


int main(int argc, char **argv){
    rclcpp::init(argc ,argv);
    // ノード作成
    rclcpp::Node::SharedPtr path_node = rclcpp::Node::make_shared("path_node");
    // パブリッシャー作成
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr path_pub;
    path_pub = path_node->create_publisher<std_msgs::msg::Int32MultiArray>("path_numbers", 10);;

    // 送信するメッセージ(未完成)
    std_msgs::msg::Int32MultiArray path_msg;
    path_msg.data = {1,2,3,4};

    path_pub->publish(path_msg);
    
    rclcpp::spin(path_node);
    //rclcpp::spin_some(path_node);
    rclcpp::shutdown();
    return 0;
}