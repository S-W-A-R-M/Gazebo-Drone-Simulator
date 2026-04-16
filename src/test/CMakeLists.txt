cmake_minimum_required(VERSION 3.8)
project(swarm_drone)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Build the node
add_executable(thermal_sensor_node src/thermal_sensor_node.cpp)
ament_target_dependencies(thermal_sensor_node rclcpp sensor_msgs)

# Install
install(TARGETS thermal_sensor_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()