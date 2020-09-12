#!/usr/bin/env python3
import rospy
from zakhar_pycore import zakhar__log as log
from zakhar_i2c_devices.moving_platform import moving_platform


if __name__ == "__main__":
    moving_platform.start(log_level=log.INFO)
