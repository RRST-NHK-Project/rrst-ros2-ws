cmake_minimum_required(VERSION 3.8)
project(damiao_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(bridge_node src/bridge_node.cpp)
target_include_directories(bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(bridge_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  bridge_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

add_executable(frame_codec_selftest src/frame_codec_selftest.cpp)
target_include_directories(frame_codec_selftest PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(frame_codec_selftest PUBLIC cxx_std_17)

add_executable(damiao_driver_node src/damiao_driver_node.cpp)
target_include_directories(damiao_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(damiao_driver_node PUBLIC cxx_std_17)
ament_target_dependencies(
  damiao_driver_node
  "rclcpp"
  "std_msgs"
)

add_executable(ps4_teleop_node src/ps4_teleop_node.cpp)
target_compile_features(ps4_teleop_node PUBLIC cxx_std_17)
ament_target_dependencies(
  ps4_teleop_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

add_executable(motor_monitor src/motor_monitor_node.cpp)
target_include_directories(motor_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(motor_monitor PUBLIC cxx_std_17)
ament_target_dependencies(
  motor_monitor
  "rclcpp"
  "std_msgs"
)

# 自分のプログラムのお手本(docs/開発ガイド.md参照)
add_executable(example_motor_control src/example_motor_control.cpp)
target_compile_features(example_motor_control PUBLIC cxx_std_17)
ament_target_dependencies(
  example_motor_control
  "rclcpp"
  "std_msgs"
)

install(TARGETS bridge_node frame_codec_selftest damiao_driver_node ps4_teleop_node motor_monitor
  example_motor_control
  DESTINATION lib/${PROJECT_NAME})

# 実機なしで試す道具・ESCの設定ツール(ros2 run damiao_ctrl fake_esp32.py などで使える)
install(PROGRAMS scripts/fake_esp32.py scripts/dm_usbcan_param.py scripts/two_motor_test.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
