cmake_minimum_required(VERSION 3.0.2)
project(action_bt)

# Add C++11 support
add_compile_options(-std=c++11)

# Find catkin and required ROS components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  msg_file
  cv_bridge
  image_transport
  geometry_msgs
)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Explicitly find yaml-cpp with pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# Declare catkin package with dependencies
catkin_package(
  CATKIN_DEPENDS roscpp actionlib msg_file cv_bridge image_transport geometry_msgs
)

# Setup include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# Declare the executable
add_executable(lane_detect_server src/lane_detect_server.cpp)

# Link libraries - explicitly include yaml-cpp
target_link_libraries(lane_detect_server
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# Add dependencies
add_dependencies(lane_detect_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Install targets
install(TARGETS lane_detect_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)