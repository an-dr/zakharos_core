from time import sleep
import rospy

import zakhar_common as com
from zakhar_msgs import srv
from zakhar_msgs import msg
from enum import IntEnum


class EgoLikeTypes(IntEnum):
    ego = 0x01
    instinct = 0x02
    reflex = 0x03


class EgoLikeNode:
    def __init__(self, name="EgoLikeNode", ego_type=EgoLikeTypes.ego):
        self.name = name
        self.ego_type = ego_type
        self.client = None
        self.subscriber = None

    def sensor_callback(self, perception_concept):
        rospy.loginfo("From SensorInterpreter: %s:%s" % (perception_concept.symbol, perception_concept.modificator))

    def _start_client(self):
        rospy.logdebug("Client \'%s\' is starting..." % self.name)
        self.client = rospy.ServiceProxy(com.names.NODE_CONCEPT_TRANSLATOR, srv.CommandConcept)
        self.subscriber = com.get.subscriber(topic_name=com.names.TOPIC_MAIN_SENSOR_INTERPRETER,
                                             data_class=msg.PerceptionConcept,
                                             callback=self.sensor_callback)
        rospy.loginfo("Client \'%s\' is ready" % self.name)

    def start(self):
        rospy.logdebug("Node \'%s\' is starting..." % self.name)
        rospy.init_node(self.name, anonymous=False)
        self._start_client()
        rospy.loginfo("[  DONE  ] Node \'%s\' is ready..." % self.name)
        self.main()

    def main(self):
        pass

    def to_will(self, symbol, modificator=""):
        rospy.loginfo("Ego is willing: %s:%s" % (symbol, modificator))
        if not self.client:
            raise rospy.ServiceException
        try:
            resp = self.client(self.ego_type, symbol, modificator)
            return resp.result
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
