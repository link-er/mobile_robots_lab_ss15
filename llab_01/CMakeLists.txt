# Declare the version of the CMake API for forward-compatibility
cmake_minimum_required(VERSION 2.8)

# Declare the name of the CMake Project
project(llab_01)

# Find and get all the information about the roscpp package
find_package(roscpp REQUIRED)

# Find Catkin
find_package(catkin REQUIRED)
# Declare this project as a catkin package
catkin_package()

# Add the local headers and the headers from roscpp
include_directories(include ${roscpp_INCLUDE_DIRS})


# Define executables target called robo_XX
add_executable(wall_follower src/wall_follower.cpp)
target_link_libraries(wall_follower ${roscpp_LIBRARIES})
#
add_executable(scanner src/scanner.cpp)
target_link_libraries(scanner ${roscpp_LIBRARIES})
#
add_executable(smooth_derivative src/smooth_derivative.cpp)
target_link_libraries(smooth_derivative ${roscpp_LIBRARIES})
#
add_executable(legs_follower src/legs_follower.cpp)
target_link_libraries(legs_follower ${roscpp_LIBRARIES})

#thats all folks

