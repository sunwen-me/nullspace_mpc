cmake_minimum_required(VERSION 3.8)
project(mpc_nav_evaluator)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mpc_eval_msgs REQUIRED)

install(PROGRAMS
  scripts/mpc_nav_evaluator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
