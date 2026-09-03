cmake_minimum_required(VERSION 3.8)
project(ros2can_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common REQUIRED)

add_executable(servo_sweep_example src/servo_sweep_example.cpp)
ament_target_dependencies(servo_sweep_example rclcpp std_msgs common)

add_executable(cubemars_position_example src/cubemars_position_example.cpp)
ament_target_dependencies(cubemars_position_example rclcpp std_msgs common)

install(TARGETS
  servo_sweep_example
  cubemars_position_example
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
