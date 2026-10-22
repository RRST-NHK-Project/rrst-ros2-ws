cmake_minimum_required(VERSION 3.8)
project(robstride_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# C++ノード: 駆動(robstride_driver_node)・PS4テレオペ・表示
add_executable(robstride_driver_node src/robstride_driver_node.cpp)
target_include_directories(robstride_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(robstride_driver_node PUBLIC cxx_std_17)
ament_target_dependencies(robstride_driver_node "rclcpp" "std_msgs")

add_executable(ps4_teleop_node src/ps4_teleop_node.cpp)
target_compile_features(ps4_teleop_node PUBLIC cxx_std_17)
ament_target_dependencies(ps4_teleop_node "rclcpp" "std_msgs" "sensor_msgs")

add_executable(motor_monitor src/motor_monitor_node.cpp)
target_include_directories(motor_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(motor_monitor PUBLIC cxx_std_17)
ament_target_dependencies(motor_monitor "rclcpp" "std_msgs")

install(TARGETS robstride_driver_node ps4_teleop_node motor_monitor
  DESTINATION lib/${PROJECT_NAME})

# rclpyのスクリプト(動作確認用)
install(PROGRAMS scripts/motion_test.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
