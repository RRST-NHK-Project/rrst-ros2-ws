cmake_minimum_required(VERSION 3.8)
project(path_ros2_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROSいぞんファイル
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#Pythonから送ってくる用
#
find_package(Python3 REQUIRED COMPONENTS Development)
find_package(pybind11 REQUIRED)

#add_executable(path_pub src/path_ros2_pub.cpp)
#ament_target_dependencies(path_pub rclcpp std_msgs)
#target_link_libraries(path_pub pybind11::embed)
#install(TARGETS path_pub DESTINATION lib/${PROJECT_NAME})

#add_executable(path_sub src/path_ros2_sub.cpp)
#ament_target_dependencies(path_sub rclcpp std_msgs)
#install(TARGETS path_sub DESTINATION lib/${PROJECT_NAME})


ament_package()
