#######################################################################################
cmake_minimum_required(VERSION 3.0.2)
#######################################################################################
project(l3xz_sweep_scanner)
#######################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)
#######################################################################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES l3xz
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)
#######################################################################################
include_directories(${catkin_INCLUDE_DIRS})
#######################################################################################
add_subdirectory(external/libsweep/current)
#######################################################################################
add_executable(${PROJECT_NAME}_node
  src/node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  pthread
  sweep
)
target_compile_options(${PROJECT_NAME}_node PRIVATE -std=c++17 -Wall -Werror -pedantic)
#######################################################################################
