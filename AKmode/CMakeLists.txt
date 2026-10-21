cmake_minimum_required(VERSION 3.8)
project(AKmode)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  src
)

# PS4コントローラ -> ros2can (serial_tx_/serial_rx_) -> CubeMars AK
add_executable(ak_gamepad_node src/ak_gamepad_node.cpp)
ament_target_dependencies(ak_gamepad_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  ak_gamepad_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
