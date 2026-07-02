cmake_minimum_required(VERSION 3.8)
project(esc_ctrl)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(esc_ctrl_node src/esc_ctrl_node.cpp)
ament_target_dependencies(esc_ctrl_node
  rclcpp
  std_msgs
)

add_executable(esc_ctrl_dev_frame src/esc_ctrl_dev_frame.cpp)
ament_target_dependencies(esc_ctrl_dev_frame
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(esc_position_pid_node src/esc_position_pid_node.cpp)
ament_target_dependencies(esc_position_pid_node
  common
  rclcpp
  std_msgs
)

install(TARGETS
  esc_ctrl_node
  esc_ctrl_dev_frame
  esc_position_pid_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
