cmake_minimum_required(VERSION 3.0.2)
project(zakhar)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
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
