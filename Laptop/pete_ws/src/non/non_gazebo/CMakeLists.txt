cmake_minimum_required(VERSION 2.8.3)
project(non_gazebo)
cmake_policy(SET CMP0054 NEW)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  #gazebo-dev
  nav_msgs
  roscpp
  std_msgs
  std_srvs
  tf
  message_generation
  gazebo_ros
)

include_directories(include ${catkin_INCLUDE_DIRS})


## Find gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf message_runtime gazebo_ros
    INCLUDE_DIRS include
    LIBRARIES
    DEPENDS GAZEBO
)
#add_executable(gazebo_ros_diff_drive src/gazebo_ros_diff_drive.cpp)
#add_dependencies(gazebo_ros_diff_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
###########
## Build ##
###########
#add_library(gazebo_ros_diff_drive src/gazebo_ros_diff_drive.cpp)
#target_link_libraries(gazebo_ros_diff_drive ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch models rviz worlds 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
