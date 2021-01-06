import rospy
import threading
import zakhar_common as com
from zakhar_msgs import msg, srv
from typing import Union


class Concept2CommandsInterpreterAbstract:
    def __init__(self, name="Concept2CommandsInterpreter"):
        self.name = name
        self.publisher = None
        self.subscriber = None
        self.server = None  # type: Union[rospy.Service, None]
        self.exec_in_progress = False
        self.data = {}

    def publish(self, target="all", message="hello_world", arg_a=0, arg_b=0, arg_c=0, arg_d=0, arg_str="."):
        to_send = msg.DeviceCmd()
        to_send.target = target
        to_send.msg = message
        to_send.argumentA = arg_a
        to_send.argumentB = arg_b
        to_send.argumentC = arg_c
        to_send.argumentD = arg_d
        to_send.argumentString = arg_str
        self.publisher.publish(to_send)
        rospy.logdebug("Send to %s: `%s`, %x, %x, %x, %x, `%s`" % (target, msg, arg_a, arg_b, arg_c, arg_d, arg_str))

    def concept_handler(self, req):
        """
        Parameters
        ----------
        req : srv.ConceptRequest
        """
        s = req.symbol
        ego_type = req.type
        rospy.loginfo("Got %s with source type: %d" % (s, ego_type))
        if not self.exec_in_progress:
            self.exec_in_progress = True
            rospy.logdebug("Object: %s; Method: %s" % (str(self), str(s)))
            method_to_call = getattr(self, s, None)
            if method_to_call is None:
                rospy.logerr("Unknown concept")
                res = "no concept"
            else:
                try:
                    rospy.logdebug("Executing...")
                    d = threading.Thread(name=s, target=method_to_call)
                    d.setDaemon(True)
                    d.start()
                    rospy.loginfo("Executed %s" % (str(s)))
                    res = "done"
                except Exception as e:
                    rospy.logerr("Error: " + str(e))
                    res = str(e)
            return srv.ConceptResponse(result=res)
        else:
            r = "Execution in process. Ignoring the request."
            rospy.loginfo(r)
            return srv.ConceptResponse(result=r)

    def sensor_callback(self, data):
        rospy.logdebug("From SensorInterpreter: %s:0x%x" % (data.perception_summary, data.argument))
        self.data[data.perception_summary] = data.argument
        rospy.logdebug(self.data)

    def start(self):
        rospy.logdebug("Node \'%s\' is starting..." % self.name)
        rospy.init_node(self.name, anonymous=False)

        self.publisher = com.get.publisher(topic_name=com.names.TOPIC_DEVICECMD, data_class=msg.DeviceCmd)
        self.server = com.get.server(name=self.name, service=srv.Concept, handle=self.concept_handler)
        self.subscriber = com.get.subscriber(topic_name=com.names.TOPIC_MAIN_SENSOR_INTERPRETER,
                                             data_class=msg.SensorConcept,
                                             callback=self.sensor_callback)

        rospy.loginfo("[  DONE  ] Node \'%s\' is ready..." % self.name)
