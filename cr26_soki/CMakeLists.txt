cmake_minimum_required(VERSION 3.8)
project(cr26_soki)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(common REQUIRED)

add_executable(welcome src/welcomeday.cpp)
ament_target_dependencies(welcome rclcpp std_msgs sensor_msgs geometry_msgs common)

add_executable(angle src/angle_test.cpp)
ament_target_dependencies(angle rclcpp std_msgs sensor_msgs geometry_msgs common)

add_executable(angle_gm src/angle_test_gm.cpp)
ament_target_dependencies(angle_gm rclcpp std_msgs sensor_msgs geometry_msgs common)

add_executable(ros2can_practice src/ros2can_practice.cpp)
ament_target_dependencies(ros2can_practice common rclcpp std_msgs sensor_msgs)

# インストール設定
install(TARGETS
  welcome
  angle
  angle_gm
  ros2can_practice
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
