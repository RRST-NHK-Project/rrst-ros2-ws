cmake_minimum_required(VERSION 3.8)
project(nr26_r1_hw_ctrl)

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

add_executable(r1_sd src/R1_SwerveDrive.cpp)
ament_target_dependencies(r1_sd rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs)

add_executable(r1_mc src/R1_MotionCtrl.cpp)
ament_target_dependencies(r1_mc rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs)


add_executable(r1_servo_cal src/R1_Servo_Calibrator.cpp)
ament_target_dependencies(r1_servo_cal rclcpp std_msgs sensor_msgs geometry_msgs)
target_include_directories(r1_servo_cal PRIVATE ${nlohmann_json_SOURCE_DIR}/single_include)

# インストール設定
install(TARGETS
  r1_sd
  r1_mc
  r1_servo_cal
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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
