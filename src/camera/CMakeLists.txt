cmake_minimum_required(VERSION 3.0.2)
project(camera)

## Find catkin and required ROS components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  std_srvs
)

## Find OpenCV
find_package(OpenCV REQUIRED)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare the video recorder executable
add_executable(recorder src/recorder.cpp)
target_link_libraries(recorder ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(camera_laptop src/camera_laptop.cpp)
target_link_libraries(camera_laptop ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(camera_laptop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(image_projector src/image_projector.cpp)
target_link_libraries(image_projector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(image_projector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Mark executables for installation
install(TARGETS 
  recorder
  camera_laptop 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)