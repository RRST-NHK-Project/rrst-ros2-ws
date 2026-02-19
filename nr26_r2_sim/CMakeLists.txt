cmake_minimum_required(VERSION 3.8)
project(nr26_r2_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# --- 実行ファイルの定義 ---
add_executable(wheel_test src/wheel_test.cpp)
ament_target_dependencies(wheel_test rclcpp std_msgs)

# --- インストール設定 ---

# 実行バイナリ
install(TARGETS wheel_test
  DESTINATION lib/${PROJECT_NAME}
)

# ディレクトリ一括コピー (config を追加しました)
install(
  DIRECTORY launch urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()