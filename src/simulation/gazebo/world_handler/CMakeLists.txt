cmake_minimum_required(VERSION 3.8)
project(world_handler)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/world_handler_node.cpp src/world_handler.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

#############
## Install ##
#############

## Mark other files for installation (e.g. launch and bag files, etc.)
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
