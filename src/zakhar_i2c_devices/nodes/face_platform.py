#!/usr/bin/env python3
import rospy
from zakhar_pycore import zakhar__log as log
from zakhar_i2c_devices.face_platform import face_platform


if __name__ == "__main__":
    face_platform.start(log_level=log.INFO)
    rospy.spin()
