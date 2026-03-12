cmake_minimum_required(VERSION 3.8)
project(path_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(path_pub src/path_ros2_pub.cpp)
ament_target_dependencies(path_pub rclcpp std_msgs)
install(TARGETS path_pub DESTINATION lib/${PROJECT_NAME})

ament_package()
