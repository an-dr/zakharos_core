cmake_minimum_required(VERSION 3.0.2)
project(zakhar_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files(DIRECTORY msg
                      FILES DeviceCmd.msg
                            EmotionParams.msg
                            PerceptionConcept.msg
                            SensorData.msg
)

add_service_files(DIRECTORY srv
                      FILES CommandConcept.srv
                            EmotionCoreDataDescriptor.srv
                            EmotionCoreWrite.srv
                            I2c.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs)
