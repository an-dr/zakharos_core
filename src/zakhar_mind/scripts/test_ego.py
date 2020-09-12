#!/usr/bin/env python3
from time import sleep
import rospy
import logging

import zakhar_common as com
import zakhar_log as log
import zakhar_i2c_devices as dev
import zakhar_mind
from zakhar_msgs import srv
from zakhar_msgs import msg

# roslib.load_manifest('your_package_name')
# roslib.load_manifest looks at your manifest.xml and places your dependencies on your Python path. DO NOT use multiple
# load_manifest calls. If you need multiple calls, it's probably because you're missing the correct dependencies
# in your manifest.


def ego_test():
    ego_like = zakhar_mind.EgoSmallResearcher(name="EgoSmallResearcher")
    ego_like.start()
    rospy.spin()


if __name__ == "__main__":
    ego_test()
