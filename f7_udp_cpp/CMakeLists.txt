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

add_executable(nr25_enc src/NR25_ENC.cpp)
ament_target_dependencies(nr25_enc rclcpp std_msgs sensor_msgs geometry_msgs )
add_executable(nr25_pr src/NR25_Parameter_Tuner.cpp)
ament_target_dependencies(nr25_pr rclcpp std_msgs sensor_msgs geometry_msgs )
add_executable(nr25_vel2js src/NR25_Vel_to_JointState.cpp)
ament_target_dependencies(nr25_vel2js rclcpp std_msgs sensor_msgs geometry_msgs )

add_executable(nr25_enc_pid src/NR25_ENC_PID.cpp)
ament_target_dependencies(nr25_enc_pid rclcpp std_msgs sensor_msgs geometry_msgs )
add_executable(nr25_pid src/NR25_PID.cpp src/include/UDP.cpp)
ament_target_dependencies(nr25_pid rclcpp std_msgs sensor_msgs geometry_msgs )

add_executable(nr25_dr_c610 src/NR25_DR_C610.cpp src/include/UDP.cpp)
ament_target_dependencies(nr25_dr_c610 rclcpp std_msgs sensor_msgs geometry_msgs )

install(TARGETS
  nr25_mr
  nr25_mr_omni
  nr25_mr_sd
  nr25_dr
  nr25_dr_omni
  nr25_dr_sd
  nr25_enc
  nr25_pr
  nr25_vel2js
  nr25_enc_pid
  nr25_pid
  nr25_dr_c610
  DESTINATION lib/${PROJECT_NAME})

ament_package()