cmake_minimum_required(VERSION 2.8.3)
project(sence_hardware)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
  dynamixel_sdk
  )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# find_package(CURL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp hardware_interface controller_manager dynamixel_sdk
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

# clients lib
#add_library(${PROJECT_NAME}_clients
#  src/rest_client.cpp
#  )

#target_link_libraries(${PROJECT_NAME}_clients
#  ${CURL_LIBRARIES}
#  )

# track hw interface lib
#add_library(${PROJECT_NAME}_interface
#  src/track_interface.cpp
#  )

#target_link_libraries(${PROJECT_NAME}_interface
#  ${PROJECT_NAME}_clients
#  )

# main hw control loop node
add_executable(${PROJECT_NAME}_node
  src/sence_hw.cpp
  )

target_link_libraries(${PROJECT_NAME}_node
  #${PROJECT_NAME}_interface
  ${catkin_LIBRARIES}
  )
