cmake_minimum_required(VERSION 3.0.2)
project(aliveos_py)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()


## Declare a catkin package
catkin_package(
 CATKIN_DEPENDS rospy
)

## Build
include_directories( include ${catkin_INCLUDE_DIRS} )
