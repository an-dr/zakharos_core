import rospy
from zakhar_msgs import msg
from .mind_classes import ReflexNode


class Node(ReflexNode):
    def sensor_callback(self, perception_concept: msg.PerceptionConcept):
        rospy.loginfo("From SensorInterpreter: %s:%s" % (perception_concept.symbol, perception_concept.modifier))
        if perception_concept.symbol == "obstacle":
            mod = perception_concept.modifier
            rospy.loginfo("Trigger! Mod: %s" % mod)
            if (("c" in mod) & 1) or (("l" in mod) and ("r" in mod)):
                self.to_will_cb("avoid", ("front"))
            else:
                if "l" in mod:
                    self.to_will_cb("avoid", ("left"))
                if "r" in mod:
                    self.to_will_cb("avoid", ("right"))


node = Node(name="ReflexAvoidCloseObjects")
