import rospy
from zakhar_msgs import msg
from .mind_classes import InstinctNode


class Node(InstinctNode):
    def sensor_callback(self, perception_concept: msg.PerceptionConcept):
        rospy.loginfo("From SensorInterpreter: %s:%s" % (perception_concept.symbol, perception_concept.modifier))
        if perception_concept.symbol == "bird":
            self.to_will_cb("basic_panic")


node = Node(name="InstinctBirdPanic")
