cmake_minimum_required(VERSION 3.8)
project(cubemars_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CUBEMARS_INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# ESP32とのシリアル送受信(serial_tx_<ID> / serial_rx_<ID>)
add_executable(cubemars_bridge_node src/bridge_node.cpp)
target_include_directories(cubemars_bridge_node PUBLIC ${CUBEMARS_INCLUDE_DIRS})
target_compile_features(cubemars_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(cubemars_bridge_node rclcpp std_msgs)

# 速度/角度指令 -> 24スロット変換と安全処理
add_executable(cubemars_driver_node src/cubemars_driver_node.cpp)
target_include_directories(cubemars_driver_node PUBLIC ${CUBEMARS_INCLUDE_DIRS})
target_compile_features(cubemars_driver_node PUBLIC cxx_std_17)
ament_target_dependencies(cubemars_driver_node rclcpp std_msgs sensor_msgs)

# PS4コントローラ(/joy) -> 速度/角度指令
add_executable(ps4_teleop_node src/ps4_teleop_node.cpp)
target_compile_features(ps4_teleop_node PUBLIC cxx_std_17)
ament_target_dependencies(ps4_teleop_node rclcpp std_msgs sensor_msgs)

# 実機なしで実行できる単体チェック(フレーム形式・単位換算)
add_executable(cubemars_selftest src/selftest.cpp)
target_include_directories(cubemars_selftest PUBLIC ${CUBEMARS_INCLUDE_DIRS})
target_compile_features(cubemars_selftest PUBLIC cxx_std_17)

install(TARGETS cubemars_bridge_node cubemars_driver_node ps4_teleop_node cubemars_selftest
  DESTINATION lib/${PROJECT_NAME})

# 実機なしでROS側を確認するための疑似ESP32
install(PROGRAMS tools/fake_esp32.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ソースに著作権ヘッダを入れていないためcopyright/cpplintはスキップする
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
