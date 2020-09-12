from time import sleep
import rospy
import logging

import zakhar_common as com
from zakhar_pycore import zakhar__log as log
import zakhar_i2c_devices as dev
import zakhar_mind
from zakhar_msgs import srv
from zakhar_msgs import msg
from enum import IntEnum

class EgoLikeTypes(IntEnum):
    ego = 0x01
    instinct = 0x02
    reflex = 0x03


class EgoLikeNode:
    def __init__(self, name="EgoLikeNode", ego_type=EgoLikeTypes.ego, log_level=log.INFO):
        self.name = name
        self.ego_type = ego_type
        self.l = log.get_logger(name=self.name, log_level=log_level)
        self.client = None
        self.subscriber = None

    def sensor_callback(self, data):
        self.l.info("From SensorInterpreter: %s:0x%x" % (data.perception_summary, data.argument))

    def _start_client(self):
        self.l.debug("Client \'%s\' is starting..." % self.name)
        self.client = rospy.ServiceProxy(
            com.names.NODE_CONCEPT_TRANSLATOR, srv.Concept)
        self.subscriber = com.get.subscriber(topic_name=com.names.TOPIC_MAIN_SENSOR_INTERPRETER,
                                        data_class=msg.SensorConcept,
                                        callback=self.sensor_callback)
        self.l.info("Client \'%s\' is ready" % self.name)

    def start(self):
        self.l.debug("Node \'%s\' is starting..." % self.name)
        rospy.init_node(self.name, anonymous=False)
        self._start_client()
        self.l.info("[  DONE  ] Node \'%s\' is ready..." % self.name)
        self.main()

    def main(self):
        pass

    def to_will(self, symbol):
        self.l.info("Ego is willing: %s" % symbol)
        if not self.client:
            raise rospy.ServiceException
        try:
            resp = self.client(self.ego_type, symbol)
            return resp.result
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
