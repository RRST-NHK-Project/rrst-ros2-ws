cmake_minimum_required(VERSION 3.8)
project(example)

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

add_executable(ps4_listener src/PS4_Listener.cpp)
ament_target_dependencies(ps4_listener rclcpp std_msgs sensor_msgs)

add_executable(udp_send src/UDP_Send.cpp src/include/UDP.cpp src/include/IP.cpp)
ament_target_dependencies(udp_send rclcpp std_msgs sensor_msgs)

add_executable(simple_pub src/Simple_Publisher.cpp)
ament_target_dependencies(simple_pub rclcpp std_msgs)

add_executable(simple_sub src/Simple_Subscriber.cpp)
ament_target_dependencies(simple_sub rclcpp std_msgs)

add_executable(usb_serial src/Simple_Subscriber.cpp)
ament_target_dependencies(usb_serial rclcpp std_msgs)

# インストール設定
install(TARGETS
  ps4_listener
  udp_send
  simple_pub
  simple_sub
  usb_serial
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
