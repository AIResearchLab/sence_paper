cmake_minimum_required(VERSION 3.10.2)
project(sence_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)

catkin_package()
