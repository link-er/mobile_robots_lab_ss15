cmake_minimum_required(VERSION 2.8)
project(drone_lab)

find_package(catkin REQUIRED cv_bridge genmsg image_transport sensor_msgs)

catkin_package()

find_package(OpenCV)

include_directories(include ${OpenCV_INCLUDE_DIRS})

# Define executables
add_executable(record_video src/record_video.cpp)
target_link_libraries(record_video ${catkin_LIBRARIES})

add_executable(analyze_image src/analyze_image.cpp)
target_link_libraries(analyze_image ${catkin_LIBRARIES})

add_executable(takeoff src/takeoff.cpp)
target_link_libraries(takeoff ${roscpp_LIBRARIES})

add_executable(fly_and_show src/fly_and_show.cpp)
target_link_libraries(fly_and_show ${catkin_LIBRARIES} ${roscpp_LIBRARIES})

add_executable(move src/move.cpp)
target_link_libraries(move ${catkin_LIBRARIES} ${roscpp_LIBRARIES})

add_executable(reset src/reset.cpp)
target_link_libraries(reset ${catkin_LIBRARIES} ${roscpp_LIBRARIES})

add_executable(land src/land.cpp)
target_link_libraries(land ${catkin_LIBRARIES} ${roscpp_LIBRARIES})

add_executable(const_height src/const_height.cpp)
target_link_libraries(const_height ${catkin_LIBRARIES} ${roscpp_LIBRARIES})

add_executable(detect_direction src/detect_direction.cpp)
target_link_libraries(detect_direction ${catkin_LIBRARIES} ${roscpp_LIBRARIES})


