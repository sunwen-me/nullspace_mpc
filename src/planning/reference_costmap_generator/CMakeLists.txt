cmake_minimum_required(VERSION 3.8)
project(reference_costmap_generator)

# parameters
option(USE_OPENMP "USE_OPENMP" ON) # ON / OFF

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# load openmp
if(USE_OPENMP)
  find_package(OpenMP REQUIRED)
  if(OpenMP_FOUND)
      message(WARNING "OpenMP found. Activate CPU acceleration.")
      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  endif()
  if(NOT OPENMP_FOUND)
      message(FATAL ERROR "Unable to find OpenMP library.")
  endif()
endif()

include_directories(
  include
)

add_library(reference_costmap_generator SHARED src/reference_costmap_generator.cpp)

ament_target_dependencies(reference_costmap_generator
  rclcpp
  grid_map_core
  grid_map_msgs
  grid_map_ros
  nav_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

## link openmp
if(USE_OPENMP)
  if (OPENMP_FOUND)
      if (TARGET OpenMP::OpenMP_CXX)
          target_link_libraries(reference_costmap_generator OpenMP::OpenMP_CXX)
      endif()
  endif()
endif()

add_executable(${PROJECT_NAME}_node src/reference_costmap_generator_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  reference_costmap_generator
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  grid_map_core
  grid_map_msgs
  grid_map_ros
  nav_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS reference_costmap_generator ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
