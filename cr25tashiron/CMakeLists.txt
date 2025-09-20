cmake_minimum_required(VERSION 3.8)
project(cr25tashiron)

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
find_package(actuator_msg REQUIRED)

add_executable(cr25_matsu src/CR25_matsu.cpp)
ament_target_dependencies(cr25_matsu rclcpp actuator_msg std_msgs sensor_msgs geometry_msgs)

add_executable(actuator_msg_publisher src/ActuatorMsgPublisher.cpp)
ament_target_dependencies(actuator_msg_publisher rclcpp actuator_msg)

add_executable(actuator_msg_subscriber src/ActuatorMsgSubscriber.cpp)
ament_target_dependencies(actuator_msg_subscriber rclcpp actuator_msg)

add_executable(actuator_msg_bridge src/ActuatorMsgBridge.cpp)
ament_target_dependencies(actuator_msg_bridge rclcpp actuator_msg)

add_executable(cr25_matsu_sw src/CR25_matsu_sw.cpp)
ament_target_dependencies(cr25_matsu_sw rclcpp actuator_msg std_msgs sensor_msgs geometry_msgs)

add_executable(cr25_debug src/cr25_debug.cpp)
ament_target_dependencies(cr25_debug rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(cr25_deg_pub src/cr25_deg_publisher.cpp)
ament_target_dependencies(cr25_deg_pub rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(cr25_num_pub src/cr25_num_publisher.cpp)
ament_target_dependencies(cr25_num_pub rclcpp std_msgs)

install(TARGETS
  cr25_matsu
  cr25_matsu_sw
  actuator_msg_publisher
  actuator_msg_subscriber
  actuator_msg_bridge
  cr25_debug
  cr25_deg_pub
  cr25_num_pub
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

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)


ament_package()
