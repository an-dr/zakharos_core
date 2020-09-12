from time import sleep
import rospy
import logging

import zakhar_common as com
from zakhar_pycore import zakhar__log as log
import zakhar_i2c_devices as dev
import zakhar_mind
from zakhar_msgs import srv
from zakhar_msgs import msg
from .ego_like import *


class InstinctBirdPanic(EgoLikeNode):
    def __init__(self, name="InstinctBirdPanic", ego_type=EgoLikeTypes.instinct, log_level=log.INFO):
        EgoLikeNode.__init__(self, name=name, ego_type=ego_type, log_level=log_level)

    def sensor_callback(self, data):
        self.l.info("From SensorInterpreter: %s:0x%x" % (data.perception_summary, data.argument))
        if data.perception_summary == "bird":
            self.to_will("basic_panic")


instinct = InstinctBirdPanic()
