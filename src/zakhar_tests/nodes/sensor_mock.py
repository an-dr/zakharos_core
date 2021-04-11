#!/usr/bin/env python3
# *************************************************************************
#
# Copyright (c) 2021 Andrei Gramakov. All rights reserved.
#
# This file is licensed under the terms of the MIT license.
# For a copy, see: https://opensource.org/licenses/MIT
#
# site:    https://agramakov.me
# e-mail:  mail@agramakov.me
#
# *************************************************************************
from time import sleep
import rospy
from zakhar_pycore import ros as zros
from zakhar_msgs import msg
import zakhar_common as com

if __name__ == '__main__':
    rospy.init_node("test_SomeSensor", anonymous=False)

    publisher_of_sensor_data = com.get.publisher(topic_name=zros.topics.SENSOR_DATA, data_class=msg.SensorData)

    to_send = msg.SensorData()
    to_send.sensor_type = "test_SomeSensor"
    to_send.valA = 1
    to_send.valB = 2
    to_send.valC = 3
    to_send.valD = 4
    to_send.valString = "Some String"
    to_send.message = "Some Message"

    while True:
        rospy.loginfo("%s published: %s, %d, %d, %d, %d, %s (%s)" %
                      ("test_SomeSensor", to_send.sensor_type, to_send.valA, to_send.valB, to_send.valC, to_send.valD,
                       to_send.valString, to_send.message))
        publisher_of_sensor_data.publish(to_send)
        # rospy.spin()
        sleep(1)
        to_send.valA += 1
        to_send.valB += 1
        to_send.valC += 1
        to_send.valD += 1
