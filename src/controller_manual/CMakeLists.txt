cmake_minimum_required(VERSION 3.0.2)
project(controller_manual)

## Find catkin and required ROS components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(joy2vel src/joy2vel.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(joy2vel
  ${catkin_LIBRARIES}
)

## Install targets
install(TARGETS joy2vel
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)