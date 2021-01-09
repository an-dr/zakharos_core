import rospy
from time import sleep
from .ego_like import EgoLikeNode, EgoLikeTypes


class InstinctAvoidCloseObjects(EgoLikeNode):
    def __init__(self, name="InstinctAvoidCloseObjects", ego_type=EgoLikeTypes.instinct):
        EgoLikeNode.__init__(self, name=name, ego_type=ego_type)

    def sensor_callback(self, data):
        rospy.loginfo("From SensorInterpreter: %s:0x%x" % (data.perception_summary, data.argument))
        if data.perception_summary == "obstacle":
            if ((data.argument >> 1) & 1) or (((data.argument >> 0) & 1) and ((data.argument >> 2) & 1)):
                self.to_will("move_avoid_front")
            else:
                if (data.argument >> 0) & 1:
                    self.to_will("move_avoid_left")
                if (data.argument >> 2) & 1:
                    self.to_will("move_avoid_right")


instinct = InstinctAvoidCloseObjects()
