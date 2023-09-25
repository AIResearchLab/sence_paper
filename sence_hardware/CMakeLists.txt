cmake_minimum_required(VERSION 2.8.3)

project(sence_hardware)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  hardware_interface
  controller_manager
  dynamixel_sdk
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs hardware_interface controller_manager dynamixel_sdk
)

catkin_python_setup()

catkin_install_python(PROGRAMS scripts/zerotalker.py scripts/onetalker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/sence_hardware_node.cpp
)

add_library(${PROJECT_NAME}_interface
  src/sence_hardware_interface.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_interface
  ${catkin_LIBRARIES}
)


