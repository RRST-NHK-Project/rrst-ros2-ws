cmake_minimum_required(VERSION 3.8)
project(teleop_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(sb_tester src/Serial_Bridge_Tester.cpp)
ament_target_dependencies(sb_tester rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(sb_host src/Serial_Bridge_Host.cpp)
ament_target_dependencies(sb_host rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(ros2can_host src/Ros2Can_Host.cpp)
ament_target_dependencies(ros2can_host common rclcpp std_msgs sensor_msgs)

# インストール設定
install(TARGETS
  sb_tester
  sb_host
  ros2can_host
  DESTINATION lib/${PROJECT_NAME})

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
