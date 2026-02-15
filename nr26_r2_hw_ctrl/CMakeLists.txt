cmake_minimum_required(VERSION 3.8)
project(nr26_r2_hw_ctrl)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(nr26_r2_move src/NR26_R2_Move.cpp)
ament_target_dependencies(nr26_r2_move rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(nr26_r2_action src/NR26_R2_Action.cpp)
ament_target_dependencies(nr26_r2_action rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(r2_md src/R2_MecanumDrive.cpp)
ament_target_dependencies(r2_md rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(r2_mc src/R2_MotionCtrl.cpp)
ament_target_dependencies(r2_mc rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(r2_tuner src/R2_Tuner.cpp)
ament_target_dependencies(r2_tuner rclcpp std_msgs sensor_msgs geometry_msgs)
target_include_directories(r2_tuner PRIVATE ${nlohmann_json_SOURCE_DIR}/single_include)

add_executable(r2_hand_xy src/R2_HandXY.cpp)
ament_target_dependencies(r2_hand_xy rclcpp std_msgs sensor_msgs geometry_msgs)
target_include_directories(r2_hand_xy PRIVATE ${nlohmann_json_SOURCE_DIR}/single_include)

# インストール設定
install(TARGETS
  nr26_r2_move
  nr26_r2_action
  r2_md
  r2_mc
  r2_tuner
  r2_hand_xy
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
