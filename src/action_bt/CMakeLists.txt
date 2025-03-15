cmake_minimum_required(VERSION 3.0.2)
project(action_bt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  msg_file
  cv_bridge
  image_transport
  geometry_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp actionlib msg_file cv_bridge image_transport geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(lane_detect_server src/lane_detect_server.cpp)
target_link_libraries(lane_detect_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(lane_detect_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})