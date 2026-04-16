cmake_minimum_required(VERSION 3.8)
project(offboard_control_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

# Build the node
add_executable(offboard_control_node src/offboard_control_node.cpp)
ament_target_dependencies(offboard_control_node rclcpp px4_msgs)

# Install
install(TARGETS offboard_control_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()