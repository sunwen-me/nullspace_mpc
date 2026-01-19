cmake_minimum_required(VERSION 3.8)
project(groundtruth_odom_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
)

add_executable(${PROJECT_NAME}_node src/groundtruth_odom_publisher_node.cpp src/groundtruth_odom_publisher.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
