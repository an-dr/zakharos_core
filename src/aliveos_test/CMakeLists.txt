cmake_minimum_required(VERSION 3.0.2)
project(aliveos_test)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  genmsg
)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS rospy)
include_directories( include ${catkin_INCLUDE_DIRS} )
