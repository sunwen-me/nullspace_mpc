cmake_minimum_required(VERSION 3.8)
project(map_visualizer)

# parameters
option(USE_OPENMP "USE_OPENMP" ON) # ON / OFF

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
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

add_library(map_visualizer SHARED src/map_visualizer.cpp)

ament_target_dependencies(map_visualizer
  rclcpp
  visualization_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

## link openmp
if(USE_OPENMP)
  if (OPENMP_FOUND)
      if (TARGET OpenMP::OpenMP_CXX)
          target_link_libraries(map_visualizer OpenMP::OpenMP_CXX)
      endif()
  endif()
endif()

add_executable(${PROJECT_NAME}_node src/map_visualizer_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  map_visualizer
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  visualization_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS map_visualizer ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
