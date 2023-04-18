cmake_minimum_required(VERSION 2.8.3)
project(ball_exclusion_layer)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(ball_exclusion_layer SHARED src/ball_exclusion_layer.cpp)
target_link_libraries(ball_exclusion_layer ${catkin_LIBRARIES})