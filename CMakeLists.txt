cmake_minimum_required(VERSION 2.8.3)
project(local_planner_student)


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
   Path.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs     
   std_msgs
   nav_msgs
 )
