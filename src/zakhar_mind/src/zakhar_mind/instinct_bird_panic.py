import rospy
from .ego_like import EgoLikeNode, EgoLikeTypes


class InstinctBirdPanic(EgoLikeNode):
    def __init__(self, name="InstinctBirdPanic", ego_type=EgoLikeTypes.instinct):
        EgoLikeNode.__init__(self, name=name, ego_type=ego_type)

    def sensor_callback(self, data):
        rospy.loginfo("From SensorInterpreter: %s:0x%x" % (data.perception_summary, data.argument))
        if data.perception_summary == "bird":
            self.to_will("basic_panic")


instinct = InstinctBirdPanic()
