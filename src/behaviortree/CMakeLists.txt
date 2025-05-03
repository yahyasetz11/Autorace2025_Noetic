cmake_minimum_required(VERSION 3.0.2)
project(behaviortree)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  msg_file
)

find_package(behaviortree_cpp_v3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp actionlib msg_file
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)

add_executable(lane_detect_client src/lane_detect_client.cpp)
target_link_libraries(lane_detect_client ${catkin_LIBRARIES})
add_dependencies(lane_detect_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(construction_client src/construction_client.cpp)
target_link_libraries(construction_client ${catkin_LIBRARIES})
add_dependencies(construction_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tunnel_nav_client src/tunnel_nav_client.cpp)
target_link_libraries(tunnel_nav_client ${catkin_LIBRARIES})
add_dependencies(tunnel_nav_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(parking_client src/parking_client.cpp)
target_link_libraries(parking_client ${catkin_LIBRARIES})
add_dependencies(parking_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(bt_main src/bt_main.cpp)
target_link_libraries(bt_main ${catkin_LIBRARIES} ${behaviortree_cpp_v3_LIBRARIES})
add_dependencies(bt_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})