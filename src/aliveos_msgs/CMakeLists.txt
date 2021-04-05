cmake_minimum_required (VERSION 3.0.2)
project (aliveos_msgs)

find_package (catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files (
    DIRECTORY
    msg
    #
    FILES
    DeviceCmd.msg
    DeviceData.msg
    EmotionParams.msg
    PerceptionConcept.msg
)

add_service_files (
    DIRECTORY
    srv
    #
    FILES
    CommandConcept.srv
    CommandConceptDescriptor.srv
    EmotionCoreDataDescriptor.srv
    EmotionCoreWrite.srv
    Hw.srv
    PerceptionConceptDescriptor.srv
)

generate_messages (DEPENDENCIES std_msgs)

catkin_package (CATKIN_DEPENDS std_msgs)
