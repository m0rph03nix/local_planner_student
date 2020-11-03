cmake_minimum_required(VERSION 2.8.3)
project(local_planner_raph)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   goalMsg.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   localGoal.srv
   PathToGoal.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs     
   std_msgs
   nav_msgs
 )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  #INCLUDE_DIRS include
#  LIBRARIES local_planner_raph #path_generator
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf message_runtime
#  DEPENDS system_lib
#)
