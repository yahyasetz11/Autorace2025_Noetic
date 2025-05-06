cmake_minimum_required(VERSION 3.0.2)
project(hsv_tuner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp cv_bridge image_transport sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_executable(hsv_tuner src/hsv_tuner.cpp)
target_link_libraries(hsv_tuner
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)