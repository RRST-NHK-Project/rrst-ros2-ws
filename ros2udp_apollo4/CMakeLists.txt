cmake_minimum_required(VERSION 3.8)
project(ros2udp_apollo4)

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
add_executable(ap4_main src/apollo4_main.cpp src/include/UDP.cpp src/include/IP.cpp)
ament_target_dependencies(ap4_main rclcpp std_msgs sensor_msgs geometry_msgs)

# インストール設定
install(TARGETS
  ap4_main

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


ament_package()