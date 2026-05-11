cmake_minimum_required(VERSION 3.8)
project(esc_ctrl)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(esc_ctrl_node src/esc_ctrl_node.cpp)
ament_target_dependencies(esc_ctrl_node
  rclcpp
  std_msgs
)

install(TARGETS
  esc_ctrl_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
