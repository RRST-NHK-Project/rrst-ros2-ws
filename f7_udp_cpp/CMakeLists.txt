cmake_minimum_required(VERSION 3.5)
project(f7_udp_cpp)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(nr25_mr src/NR25_MR.cpp src/include/UDP.cpp)
ament_target_dependencies(nr25_mr rclcpp std_msgs sensor_msgs geometry_msgs)
add_executable(nr25_mr_omni src/NR25_MR_Omni.cpp src/include/UDP.cpp)
ament_target_dependencies(nr25_mr_omni rclcpp std_msgs sensor_msgs geometry_msgs)
add_executable(nr25_mr_sd src/NR25_MR_SwerveDrive.cpp src/include/UDP.cpp)
ament_target_dependencies(nr25_mr_sd rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(nr25_dr src/NR25_DR.cpp src/include/UDP.cpp)
ament_target_dependencies(nr25_dr rclcpp std_msgs sensor_msgs geometry_msgs)
add_executable(nr25_dr_omni src/NR25_DR_Omni.cpp src/include/UDP.cpp)
ament_target_dependencies(nr25_dr_omni rclcpp std_msgs sensor_msgs geometry_msgs)
add_executable(nr25_dr_sd src/NR25_DR_SwerveDrive.cpp src/include/UDP.cpp)
ament_target_dependencies(nr25_dr_sd rclcpp std_msgs sensor_msgs geometry_msgs)

install(TARGETS
  nr25_mr
  nr25_mr_omni
  nr25_mr_sd
  nr25_dr
  nr25_dr_omni
  nr25_dr_sd
  DESTINATION lib/${PROJECT_NAME})

ament_package()