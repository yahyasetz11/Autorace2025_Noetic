cmake_minimum_required(VERSION 3.0.2)
project(hsv_reconfigure)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
)

## Find external packages
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

## Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(
  cfg/HSVTuner.cfg
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp dynamic_reconfigure cv_bridge image_transport sensor_msgs std_msgs
)

## Build
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

## Declare the executable
add_executable(hsv_reconfigure_tuner src/hsv_reconfigure_tuner.cpp)

## Add dependencies to the executable
add_dependencies(hsv_reconfigure_tuner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link the executable against
target_link_libraries(hsv_reconfigure_tuner
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS hsv_reconfigure_tuner
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)