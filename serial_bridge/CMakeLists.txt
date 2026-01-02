cmake_minimum_required(VERSION 3.8)
project(serial_bridge)

set(CMAKE_CXX_STANDARD 17)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

add_executable(serial_bridge_node
  src/main.cpp
  src/port_scanner.cpp
  src/bridge_node.cpp
)

ament_target_dependencies(serial_bridge_node
  rclcpp
  std_msgs
)

install(TARGETS
  serial_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
