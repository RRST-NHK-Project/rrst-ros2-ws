#include "serial_bridge/bridge_node.hpp"
#include "serial_bridge/port_scanner.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // 全マイコンの (ID → ポート) を検出
    auto devices = detect_serial_devices();

    if (devices.empty()) {
        std::cerr << "No serial devices found\n";
        return 1;
    }

    rclcpp::executors::MultiThreadedExecutor executor;
    std::vector<std::shared_ptr<SerialBridgeNode>> nodes;

    for (auto &item : devices) {
        uint8_t id = item.first;
        const std::string &port = item.second;

        auto node = std::make_shared<SerialBridgeNode>(id, port);
        nodes.push_back(node);
        executor.add_node(node);
    }

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
