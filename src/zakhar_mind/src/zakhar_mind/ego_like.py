import rospy
import json
import zakhar_common as com
from zakhar_msgs import srv
from zakhar_msgs import msg
from zakhar_pycore import constants as zc
from enum import IntEnum


class EgoLikeTypes(IntEnum):
    ego = 0x01
    instinct = 0x02
    reflex = 0x03


class EgoLikeNode:
    def __init__(self, name="EgoLikeNode", ego_type=EgoLikeTypes.ego):
        self.name = name
        self.ego_type = ego_type
        self.client_CommandConcept = None
        self.subscriber_PerceptionConcept = None

    def sensor_callback(self, perception_concept: msg.PerceptionConcept):
        rospy.loginfo("From SensorInterpreter: %s:%s" % (perception_concept.symbol, perception_concept.modifier))

    def emotion_callback(self, params: msg.EmotionParams):
        params_dict = json.loads(params.params_json)
        rospy.loginfo(f"Params: ${str(params_dict)}")

    def _init_communication(self):
        rospy.logdebug("Client \'%s\' is starting..." % self.name)
        self.client_CommandConcept = rospy.ServiceProxy(zc.ROS.SERVICES.CONCEPT_TO_COMMAND, srv.CommandConcept)
        self.subscriber_PerceptionConcept = com.get.subscriber(topic_name=zc.ROS.TOPICS.MAIN_SENSOR_INTERPRETER,
                                                               data_class=msg.PerceptionConcept,
                                                               callback=self.sensor_callback)
        self.subscriber_EmotionParams = com.get.subscriber(topic_name=zc.ROS.TOPICS.EMOTION_PARAMS,
                                                           data_class=msg.EmotionParams,
                                                           callback=self.emotion_callback)
        rospy.loginfo("Client \'%s\' is ready" % self.name)

    def start(self):
        rospy.logdebug("Node \'%s\' is starting..." % self.name)
        rospy.init_node(self.name, anonymous=False)
        self._init_communication()
        rospy.loginfo("[  DONE  ] Node \'%s\' is ready..." % self.name)
        self.main()

    def main(self):
        pass

    def to_will(self, symbol, modifier=()):
        rospy.loginfo("Ego is willing: %s:%s" % (symbol, str(modifier)))
        if not self.client_CommandConcept:
            raise rospy.ServiceException
        try:
            resp = self.client_CommandConcept(self.ego_type, symbol, str(modifier))
            return resp.result
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
