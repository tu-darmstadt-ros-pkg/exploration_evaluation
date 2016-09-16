cmake_minimum_required(VERSION 2.8.3)
project(exploration_evaluation)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
  task_msgs
  flexbe_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES exploration_evaluation
#  CATKIN_DEPENDS hector_worldmodel_msgs nav_msgs rospy sar_msgs task_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
