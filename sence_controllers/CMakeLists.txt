cmake_minimum_required(VERSION 3.10.2)
project(sence_controllers)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  actionlib_msgs
  message_generation
)

add_action_files(
  FILES
  StaticPose.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
