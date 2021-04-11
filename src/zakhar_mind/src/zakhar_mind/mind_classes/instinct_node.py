from .generic_mind_node import GenericMindNode, node_types
import rospy
from aliveos_msgs import msg


class InstinctNode(GenericMindNode):
    def __init__(self, name):
        super().__init__(name=name, node_type=node_types.INSTINCT_NODE)

    def sensor_callback(self, perception_concept: msg.PerceptionConcept):
        rospy.loginfo("From SensorInterpreter: %s:%s" %
                      (perception_concept.symbol, perception_concept.modifier))
