cmake_minimum_required(VERSION 3.8)
project(joy_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

add_executable(joy_controller_node src/joy_controller_node.cpp src/joy_controller.cpp)

ament_target_dependencies(joy_controller_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS joy_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
