cmake_minimum_required(VERSION 3.8)
project(serial_rx_unwrap)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(serial_rx_unwrap_node src/serial_rx_unwrap_node.cpp)
target_include_directories(serial_rx_unwrap_node PUBLIC include)
ament_target_dependencies(serial_rx_unwrap_node
  rclcpp
  std_msgs
)

install(TARGETS
  serial_rx_unwrap_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_counter_unwrapper test/test_counter_unwrapper.cpp)
  target_include_directories(test_counter_unwrapper PUBLIC include)
endif()

ament_package()
