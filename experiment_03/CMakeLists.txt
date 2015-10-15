cmake_minimum_required(VERSION 2.8.3)
project(experiment_03)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES experiment_03
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(robo_3_0 src/robo_3_0.cpp)
add_executable(robo_3_1 src/robo_3_1.cpp)
target_link_libraries(robo_3_0 ${roscpp_LIBRARIES})
target_link_libraries(robo_3_1 ${roscpp_LIBRARIES})

