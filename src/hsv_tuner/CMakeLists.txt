cmake_minimum_required(VERSION 3.0.2)
project(hsv_tuner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp cv_bridge image_transport sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(hsv_tuner src/hsv_tuner.cpp)
target_link_libraries(hsv_tuner
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)