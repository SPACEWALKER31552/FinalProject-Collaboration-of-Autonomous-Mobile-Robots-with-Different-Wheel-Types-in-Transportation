cmake_minimum_required(VERSION 2.8.3)
project(holo_description)
find_package(catkin REQUIRED COMPONENTS
robot_state_publisher
urdf
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES holo_description
CATKIN_DEPENDS robot_state_publisher urdf
#  DEPENDS system_lib
)
include_directories(
# include
${catkin_INCLUDE_DIRS}
)
