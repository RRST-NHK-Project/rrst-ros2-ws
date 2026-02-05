cmake_minimum_required(VERSION 3.8)
project(ros2esp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 自動でインストール
include(FetchContent)
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(mros_wip src/mROS_WIP.cpp)
ament_target_dependencies(mros_wip rclcpp std_msgs sensor_msgs)

add_executable(mb_tester src/MB_TESTER.cpp)
ament_target_dependencies(mb_tester rclcpp std_msgs sensor_msgs)

add_executable(nr25_mr_sd src/NR25_MR_SwerveDrive.cpp)
ament_target_dependencies(nr25_mr_sd rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(nr26_omni src/NR26_omni.cpp)
ament_target_dependencies(nr26_omni rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(nr26_omni_fb src/NR26_omni_FB.cpp)
ament_target_dependencies(nr26_omni_fb rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(nr26_tuner src/NR26_R2_Tuner.cpp)
ament_target_dependencies(nr26_tuner rclcpp std_msgs sensor_msgs geometry_msgs)
target_include_directories(nr26_tuner PRIVATE ${nlohmann_json_SOURCE_DIR}/single_include)

add_executable(nr26_r2_move src/NR26_R2_move.cpp)
ament_target_dependencies(nr26_r2_move rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(pid_wip src/PID_WIP.cpp)
ament_target_dependencies(pid_wip rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(pid_tuner src/PID_Tuner.cpp)
ament_target_dependencies(pid_tuner rclcpp std_msgs sensor_msgs geometry_msgs)
target_include_directories(pid_tuner PRIVATE ${nlohmann_json_SOURCE_DIR}/single_include)

add_executable(cr25_fes src/CR25_FES.cpp)
ament_target_dependencies(cr25_fes rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(dokusute_move src/dokusute_move.cpp)
ament_target_dependencies(dokusute_move rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(sb_tester src/Serial_Bridge_Tester.cpp)
ament_target_dependencies(sb_tester rclcpp std_msgs sensor_msgs geometry_msgs)

# インストール設定
install(TARGETS
  mros_wip
  mb_tester
  nr25_mr_sd
  nr26_omni
  nr26_omni_fb
  nr26_tuner
  cr25_fes
  pid_wip
  pid_tuner
  dokusute_move
  dokusute_tuner
  sb_tester
  nr26_r2_move
  sb_host
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
