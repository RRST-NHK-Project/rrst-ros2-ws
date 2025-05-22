cmake_minimum_required(VERSION 3.8)
project(ros2udp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# figletがインストールされていない場合、警告を表示
if(NOT DEFINED ENV{FIGLET_CHECKED})
  execute_process(
    COMMAND which figlet
    RESULT_VARIABLE figlet_installed
  )
  if(NOT figlet_installed EQUAL 0)
    message(WARNING "Please install 'figlet' with the following command. \nsudo apt install figlet")
  endif()
  set(ENV{FIGLET_CHECKED} 1)
endif()

# 自動でインストール
include(FetchContent)
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


# ターゲットの定義と依存関係の追加
add_executable(nr25_mr src/NR25_MR.cpp src/include/UDP.cpp src/include/IP.cpp)
ament_target_dependencies(nr25_mr rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(nr25_mr_omni src/NR25_MR_Omni.cpp src/include/UDP.cpp)
ament_target_dependencies(nr25_mr_omni rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(nr25_mr_sd src/NR25_MR_SwerveDrive.cpp src/include/UDP.cpp src/include/IP.cpp)
ament_target_dependencies(nr25_mr_sd rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(nr25_dr src/NR25_DR.cpp src/include/UDP.cpp src/include/IP.cpp)
ament_target_dependencies(nr25_dr rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(nr25_dr_omni src/NR25_DR_Omni.cpp src/include/UDP.cpp)
ament_target_dependencies(nr25_dr_omni rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(nr25_dr_sd src/NR25_DR_SwerveDrive.cpp src/include/UDP.cpp src/include/IP.cpp)
ament_target_dependencies(nr25_dr_sd rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(nr25_enc src/NR25_ENC.cpp)
ament_target_dependencies(nr25_enc rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(nr25_pr src/NR25_Parameter_Tuner.cpp)
ament_target_dependencies(nr25_pr rclcpp std_msgs sensor_msgs geometry_msgs)
target_include_directories(nr25_pr PRIVATE ${nlohmann_json_SOURCE_DIR}/single_include)

add_executable(nr25_vel2js src/NR25_Vel_to_JointState.cpp)
ament_target_dependencies(nr25_vel2js rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(nr25_enc_pid src/NR25_ENC_PID.cpp)
ament_target_dependencies(nr25_enc_pid rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(nr25_pid src/NR25_PID.cpp src/include/UDP.cpp)
ament_target_dependencies(nr25_pid rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(nr25_dr_c610 src/NR25_DR_C610.cpp src/include/UDP.cpp src/include/IP.cpp)
ament_target_dependencies(nr25_dr_c610 rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(nr25_mr_servo_cal src/NR25_MR_Servo_Calibrator.cpp)
ament_target_dependencies(nr25_mr_servo_cal rclcpp std_msgs sensor_msgs geometry_msgs)
target_include_directories(nr25_mr_servo_cal PRIVATE ${nlohmann_json_SOURCE_DIR}/single_include)

add_executable(nr25_dr_servo_cal src/NR25_DR_Servo_Calibrator.cpp)
ament_target_dependencies(nr25_dr_servo_cal rclcpp std_msgs sensor_msgs geometry_msgs)
target_include_directories(nr25_dr_servo_cal PRIVATE ${nlohmann_json_SOURCE_DIR}/single_include)

add_executable(nr25_dr_pr src/NR25_DR_Parameter_Tuner.cpp)
ament_target_dependencies(nr25_dr_pr rclcpp std_msgs sensor_msgs geometry_msgs)
target_include_directories(nr25_dr_pr PRIVATE ${nlohmann_json_SOURCE_DIR}/single_include)

add_executable(nr25_dr_hcsr04 src/NR25_DR_HCSR04.cpp)
ament_target_dependencies(nr25_dr_hcsr04 rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(mb_tester src/MB_TESTER.cpp src/include/UDP.cpp)
ament_target_dependencies(mb_tester rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(ld19_fs src/LD19_FrontScan.cpp)
ament_target_dependencies(ld19_fs rclcpp std_msgs sensor_msgs geometry_msgs)


add_executable(pid_wip src/PID_WIP.cpp src/include/UDP.cpp src/include/IP.cpp)
ament_target_dependencies(pid_wip rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(pid_tuner src/PID_Tuner.cpp)
ament_target_dependencies(pid_tuner rclcpp std_msgs sensor_msgs geometry_msgs)
target_include_directories(pid_tuner PRIVATE ${nlohmann_json_SOURCE_DIR}/single_include)

add_executable(nr25_led src/TeapLED_test.cpp src/include/UDP.cpp src/include/IP.cpp)
ament_target_dependencies(nr25_led rclcpp std_msgs sensor_msgs geometry_msgs)

# インストール設定
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
  nr25_mr_servo_cal
  nr25_dr_servo_cal
  nr25_dr_pr
  nr25_dr_hcsr04
  mb_tester
  ld19_fs
  pid_wip
  pid_tuner
  nr25_led

  DESTINATION lib/${PROJECT_NAME})

# figletがインストールされていない場合、警告を表示
execute_process(
  COMMAND dpkg -s figlet
  RESULT_VARIABLE figlet_installed
  OUTPUT_QUIET
  ERROR_QUIET
)

if(NOT figlet_installed EQUAL 0)
  message(WARNING "Please install 'figlet' with the following command. \nsudo apt install figlet")
endif()

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
