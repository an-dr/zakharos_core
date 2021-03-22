import rospy
import json
from zakhar_msgs import srv
from zakhar_msgs import msg
from zakhar_pycore import ros as zros
from . import node_types
from threading import Lock, Thread


class GenericMindNode:
    def __init__(self, name="GenericMindNode", node_type=node_types.GENERIC_NODE):
        self.name = name
        self.node_type = node_type
        self.client_CommandConcept = None
        self.subscriber_PerceptionConcept = None
        self.sensor_cb_lock = Lock()
        self.sensor_cb_thread = None

    def sensor_callback(self, perception_concept: msg.PerceptionConcept):
        rospy.loginfo("From SensorInterpreter: %s:%s" % (perception_concept.symbol, perception_concept.modifier))

    def _sensor_callback_thread(self, perception_concept: msg.PerceptionConcept):
        rospy.loginfo("sensor_callback thread")
        if self.sensor_cb_lock.acquire(blocking=False):
            self.sensor_callback(perception_concept)
            self.sensor_cb_lock.release()

    def _sensor_callback(self, perception_concept: msg.PerceptionConcept):
        if self.sensor_cb_lock.locked():
            rospy.loginfo("sensor_callback is busy!")
        else:
            self.sensor_cb_thread = Thread(target=self._sensor_callback_thread, args=(perception_concept,), daemon=True)
            self.sensor_cb_thread.start()

    def emotion_callback(self, params: msg.EmotionParams):
        params_dict = json.loads(params.params_json)
        rospy.loginfo(f"Params: ${str(params_dict)}")

    def _init_communication(self):
        rospy.logdebug("Client \'%s\' is starting..." % self.name)
        self.client_CommandConcept = rospy.ServiceProxy(zros.services.CONCEPT_TO_COMMAND, srv.CommandConcept)
        self.subscriber_PerceptionConcept = zros.get.subscriber(topic_name=zros.topics.MAIN_SENSOR_INTERPRETER,
                                                                data_class=msg.PerceptionConcept,
                                                                callback=self._sensor_callback)
        self.subscriber_EmotionParams = zros.get.subscriber(topic_name=zros.topics.EMOTION_PARAMS,
                                                            data_class=msg.EmotionParams,
                                                            callback=self.emotion_callback)
        rospy.loginfo("Client \'%s\' is ready" % self.name)

    def start(self):
        rospy.logdebug("Node \'%s\' is starting..." % self.name)
        rospy.init_node(self.name, anonymous=False)
        self._init_communication()
        rospy.loginfo("[  DONE  ] Node \'%s\' is ready..." % self.name)

    def to_will_cb(self, symbol: str, modifier: tuple = ()) -> str:
        """
        Sends concept requests from callbacks
        """
        rospy.loginfo("The mind is willing: %s:%s" % (symbol, str(modifier)))
        try:
            resp = self.client_CommandConcept(self.node_type, symbol, str(modifier))
            rospy.logwarn("Response for %s: %s" % (symbol, resp))
            return resp.result
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def __call__(self):
        self.start()
