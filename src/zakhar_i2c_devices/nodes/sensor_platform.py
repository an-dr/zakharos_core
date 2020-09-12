#!/usr/bin/env python3
import rospy
from zakhar_pycore import zakhar__log as log
from zakhar_i2c_devices.sensor_platform import sensor_platform


if __name__ == "__main__":
    sensor_platform.start(log_level=log.INFO)
