#include "serial_bridge/bridge_node.hpp"
#include "serial_bridge/port_scanner.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto debug_node = std::make_shared<rclcpp::Node>("serial_bridge_status");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(debug_node);

    auto devices = detect_serial_devices();

    if (devices.empty()) {
        RCLCPP_WARN(debug_node->get_logger(), "No serial devices found");
        executor.spin(); // ← debug_node は生き続ける
        rclcpp::shutdown();
        return 0;
    }

    std::vector<std::shared_ptr<SerialBridgeNode>> nodes;
    for (auto &item : devices) {
        auto node = std::make_shared<SerialBridgeNode>(item.first, item.second);
        nodes.push_back(node);
        executor.add_node(node);
    }

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
