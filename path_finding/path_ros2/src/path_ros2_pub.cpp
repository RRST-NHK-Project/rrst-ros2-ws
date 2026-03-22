// NHK2026ロボコン
// 経路->ROS2のパブリッシャー
// 2026/03/19ボツ

// python->C++
#include <pybind11/embed.h>
namespace py = pybind11;
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

    // 送信するメッセージ(Python_move関数から受取)
    py::scoped_interpreter guard{}; // Pythonインタープリタの初期化

    py::module_ path_system_move = py::module_::import("path_system_move");
    py::object Movement = path_system_move.attr("Movement");
    py::object generate_moves = Movement.attr("generate_moves")(); // path_system_move.pyのMovementクラスのgenerate_moves関数を取得

    std_msgs::msg::Int32MultiArray path_msg;
    path_msg.data = generate_moves.cast<std::vector<int>>(); // Pythonー＞C++の変換
    path_pub->publish(path_msg);
    
    rclcpp::spin(path_node);
    //rclcpp::spin_some(path_node);
    rclcpp::shutdown();
    return 0;
}