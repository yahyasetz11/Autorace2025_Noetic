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

## Specify libraries to link against
target_link_libraries(recorder
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

## Add dependencies to ensure messages are built first
add_dependencies(recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Mark executables for installation
install(TARGETS recorder
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)