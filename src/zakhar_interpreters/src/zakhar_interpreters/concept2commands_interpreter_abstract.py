import rospy
import threading
import zakhar_common as com
from zakhar_msgs import msg, srv
from typing import Union


class Concept2CommandsInterpreterAbstract:
    def __init__(self, name="Concept2CommandsInterpreter"):
        self.name = name
        self.publisher_to_devices = None
        self.subscriber_to_perception_concepts = None
        self.server_of_command_concepts = None  # type: Union[rospy.Service, None]
        self.exec_in_progress = False
        self.data = {}

    def publish(self, cmd: int, arg: int = 0, target: str = "all", message: str = ""):
        to_send = msg.DeviceCmd()
        to_send.target = target
        to_send.cmd = cmd
        to_send.arg = arg
        to_send.message = message
        self.publisher_to_devices.publish(to_send)
        rospy.logdebug("Send to %s: 0x%x(0x%x). [%s]" % (target, cmd, arg, message))

    def command_concept_handler(self, req: srv.CommandConceptRequest) -> srv.CommandConceptResponse:
        concept = req.symbol
        mod = req.modificator
        ego_type = req.ego_type
        rospy.loginfo("Got %s with source type: %d" % (concept, ego_type))

        if not self.exec_in_progress:
            self.exec_in_progress = True
            rospy.logdebug("Object: %s; Method: %s" % (str(self), str(concept)))

            method_to_call = getattr(self, concept, None)
            if method_to_call is None:
                rospy.logerr("Unknown concept")
                res = "no concept"
            else:
                try:
                    rospy.logdebug("Executing...")
                    d = threading.Thread(name=concept, target=method_to_call, kwargs={"modificator": mod})
                    d.setDaemon(True)
                    d.start()
                    rospy.loginfo("Executed %s" % (str(concept)))
                    res = "done"
                except Exception as e:
                    rospy.logerr("Error: " + str(e))
                    res = str(e)
            return srv.CommandConceptResponse(result=res)
        else:
            r = "Execution in process. Ignoring the request."
            rospy.loginfo(r)
            return srv.CommandConceptResponse(result=r)

    def sensor_callback(self, data: msg.PerceptionConcept):
        rospy.logdebug("From SensorInterpreter: %s:%s" % (data.symbol, data.modificator))
        self.data[data.symbol] = data.modificator
        rospy.logdebug(self.data)

    def start(self):
        rospy.logdebug("Node \'%s\' is starting..." % self.name)
        rospy.init_node(self.name, anonymous=False)

        self.publisher_to_devices = com.get.publisher(topic_name=com.names.TOPIC_DEVICECMD, data_class=msg.DeviceCmd)
        self.server_of_command_concepts = com.get.server(name=self.name,
                                                         service=srv.CommandConcept,
                                                         handle=self.command_concept_handler)
        self.subscriber_to_perception_concepts = com.get.subscriber(topic_name=com.names.TOPIC_MAIN_SENSOR_INTERPRETER,
                                                                    data_class=msg.PerceptionConcept,
                                                                    callback=self.sensor_callback)

        rospy.loginfo("[  DONE  ] Node \'%s\' is ready..." % self.name)
