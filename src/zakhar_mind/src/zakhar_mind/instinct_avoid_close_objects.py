import rospy
from zakhar_msgs import msg
from .ego_like import EgoLikeNode, EgoLikeTypes


class InstinctAvoidCloseObjects(EgoLikeNode):
    def __init__(self, name="InstinctAvoidCloseObjects", ego_type=EgoLikeTypes.instinct):
        EgoLikeNode.__init__(self, name=name, ego_type=ego_type)

    def sensor_callback(self, perception_concept: msg.PerceptionConcept):
        rospy.loginfo("From SensorInterpreter: %s:%s" % (perception_concept.symbol, perception_concept.modificator))
        if perception_concept.symbol == "obstacle":
            mod = perception_concept.modificator
            rospy.loginfo("Trigger! Mod: %s" % mod)
            if (("c" in mod) & 1) or (("l" in mod) and ("r" in mod)):
                self.to_will("move_avoid_front")
            else:
                if "l" in mod:
                    self.to_will("move_avoid_left")
                if "r" in mod:
                    self.to_will("move_avoid_right")


instinct = InstinctAvoidCloseObjects()
