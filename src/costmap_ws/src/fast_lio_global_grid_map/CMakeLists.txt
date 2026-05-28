cmake_minimum_required(VERSION 3.5.1)
project(fast_lio_global_grid_map)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)

find_package(catkin REQUIRED COMPONENTS
  grid_map_core
  grid_map_msgs
  grid_map_ros
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    grid_map_core
    grid_map_msgs
    grid_map_ros
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
  DEPENDS
    PCL
    Eigen3
)

include_directories(
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(global_grid_map_node
  src/global_grid_map_node.cpp
)

add_dependencies(global_grid_map_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(global_grid_map_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(
  TARGETS global_grid_map_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    config
    launch
    rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
