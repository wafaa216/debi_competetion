cmake_minimum_required(VERSION 3.0.2)
project(ball_detection_msg)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  BallPosition.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)