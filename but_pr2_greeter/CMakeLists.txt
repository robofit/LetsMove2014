cmake_minimum_required(VERSION 2.8.3)
project(but_pr2_greeter)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_commander
  moveit_msgs
  rospy
  tf
  roslint
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

roslint_cpp()
