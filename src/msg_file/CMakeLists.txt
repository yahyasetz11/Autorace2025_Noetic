cmake_minimum_required(VERSION 2.8.3)
project(msg_file)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  genmsg
  actionlib_msgs)

## Declare ROS messages and services
add_message_files(
  DIRECTORY msg 
  FILES
  Detection.msg
)

add_service_files(
  DIRECTORY srv 
  FILES )

add_action_files(
  DIRECTORY action
  FILES 
  LaneDetect.action
  Construction.action
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS 
    message_runtime 
    std_msgs 
    actionlib_msgs)

## Build
include_directories(include ${catkin_INCLUDE_DIRS})