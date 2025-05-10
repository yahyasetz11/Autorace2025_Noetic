cmake_minimum_required(VERSION 3.0.2)
project(scan_debug)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  dynamic_reconfigure
)

# Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(
  cfg/ScanDebug.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_debug_node src/scan_debug_node.cpp)
add_dependencies(scan_debug_node ${PROJECT_NAME}_gencfg)
target_link_libraries(scan_debug_node ${catkin_LIBRARIES})