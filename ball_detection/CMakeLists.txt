cmake_minimum_required(VERSION 3.0.2)
project(ball_detection)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(ball_detection_node src/ball_detection_node.cpp)

target_link_libraries(ball_detection_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)