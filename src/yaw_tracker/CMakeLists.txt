cmake_minimum_required(VERSION 3.0.2)
project(yaw_tracker)

## Find catkin and required system dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  msg_file
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(yaw_tracker_node src/yaw_tracker_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(yaw_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(yaw_tracker_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Mark executables for installation
install(TARGETS yaw_tracker_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)