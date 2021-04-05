cmake_minimum_required(VERSION 3.0.2)
project(aliveos_middle)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    aliveos_msgs
    genmsg
    roscpp
    rospy
    std_msgs
)

## Python
catkin_python_setup()

## CPP
catkin_package(CATKIN_DEPENDS rospy roscpp)
set(ALIVEOS_EMOTION_CORE_NODE "aliveos_emotion_core_node")
set (SRC "cpp/src/main.cpp" "cpp/src/aliveos_emotion_core_node.cpp")
add_executable (${ALIVEOS_EMOTION_CORE_NODE} ${SRC})
include ("cpp/lib/r_giskard_EmotionCore/cmake/AnimalEmotionCore.cmake")
include_directories ("cpp/include" "cpp/lib/nlohmann_json" ${catkin_INCLUDE_DIRS} ${AnimalEmotionCore_INCLUDE_DIRS})
target_link_libraries (${ALIVEOS_EMOTION_CORE_NODE} ${catkin_LIBRARIES} "AnimalEmotionCore")
target_compile_options (${ALIVEOS_EMOTION_CORE_NODE} PRIVATE -Wno-psabi -Wno-format) # -Wno-psabi suppresses notes of json library (apparently arm-only)
