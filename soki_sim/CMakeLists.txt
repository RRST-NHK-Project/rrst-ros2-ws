cmake_minimum_required(VERSION 3.8)
project(soki_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/motor_mixer_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motor_mixer_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
