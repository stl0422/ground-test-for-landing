cmake_minimum_required(VERSION 3.5.1)
project(safeland)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_core
  grid_map_ros
  grid_map_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    grid_map_core
    grid_map_ros
    grid_map_msgs
)

include_directories(
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(safeland_node
  src/safeland_node.cpp
)

add_dependencies(safeland_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(safeland_node
  ${catkin_LIBRARIES}
)

install(
  TARGETS safeland_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    config
    launch
    rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
