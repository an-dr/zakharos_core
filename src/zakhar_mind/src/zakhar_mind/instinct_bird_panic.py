import rospy
from zakhar_msgs import msg
from .ego_like import EgoLikeNode, EgoLikeTypes


class InstinctBirdPanic(EgoLikeNode):
    def __init__(self, name="InstinctBirdPanic", ego_type=EgoLikeTypes.instinct):
        EgoLikeNode.__init__(self, name=name, ego_type=ego_type)

    def sensor_callback(self, perception_concept: msg.PerceptionConcept):
        rospy.loginfo("From SensorInterpreter: %s:%s" % (perception_concept.symbol, perception_concept.modificator))
        if perception_concept.symbol == "bird":
            self.to_will("basic_panic")


instinct = InstinctBirdPanic()
