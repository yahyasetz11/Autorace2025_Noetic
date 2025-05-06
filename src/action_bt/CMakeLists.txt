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
  tf
  move_base_msgs
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
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# Declare the executable
add_executable(lane_detect_server src/lane_detect_server.cpp)
target_link_libraries(lane_detect_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(lane_detect_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(construction_server src/construction_server.cpp)
target_link_libraries(construction_server ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(construction_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tunnel_nav_server src/tunnel_nav_server.cpp)
target_link_libraries(tunnel_nav_server ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(tunnel_nav_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(move_server src/move_server.cpp)
target_link_libraries(move_server ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(move_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(rotate_server src/rotate_server.cpp)
target_link_libraries(rotate_server ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(rotate_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(align_server src/align_server.cpp)
target_link_libraries(align_server ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(align_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(parking_server src/parking_server.cpp)
target_link_libraries(parking_server ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(parking_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(cross_walk_server src/cross_walk_server.cpp)
target_link_libraries(cross_walk_server ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(cross_walk_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Install targets
install(TARGETS 
  lane_detect_server
  construction_server
  tunnel_nav_server
  move_server 
  rotate_server
  align_server
  parking_server
  cross_walk_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)