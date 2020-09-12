#!/usr/bin/env python3
import rospy
from zakhar_mind.sensor_interpreter import sensor_interpreter


if __name__ == '__main__':
    sensor_interpreter.start()
    rospy.spin()