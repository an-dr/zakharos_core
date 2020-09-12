import rospy
import threading
import zakhar_common as com
from zakhar_pycore import zakhar__log as log
import zakhar_i2c_devices as dev
from zakhar_msgs import msg, srv

class ConceptTranslatorAbstract:
    def __init__(self, name="ConceptTranslatorAbstract", log_level=log.INFO):
        self.name = name
        self.l = log.get_logger(name=self.name, log_level=log_level)
        self.publisher = None
        self.subscriber = None
        self.server = None  # type: rospy.Service
        self.exec_in_progress = False
        self.data = {}


    def publish(self,
                target="all",
                message="hello_world",
                arg_a=0,
                arg_b=0,
                arg_c=0,
                arg_d=0,
                arg_str="."):
        to_send = msg.DeviceCmd()
        to_send.target = target
        to_send.msg = message
        to_send.argumentA = arg_a
        to_send.argumentB = arg_b
        to_send.argumentC = arg_c
        to_send.argumentD = arg_d
        to_send.argumentString = arg_str
        self.publisher.publish(to_send)
        self.l.debug("Send to %s: `%s`, %x, %x, %x, %x, `%s`" %
                    (target, msg, arg_a, arg_b, arg_c, arg_d, arg_str))

    def concept_handler(self, req):
        """
        Parameters
        ----------
        req : srv.ConceptRequest
        """
        s = req.symbol
        ego_type = req.type
        self.l.info("Got %s with source type: %d" % (s, ego_type))
        if not self.exec_in_progress:
            self.exec_in_progress = True
            self.l.debug("Object: %s; Method: %s" % (str(self), str(s)))
            method_to_call = getattr(self, s, None)
            if method_to_call is None:
                self.l.error("Unknown concept")
                res = "no concept"
            else:
                try:
                    self.l.debug("Executing...")
                    d = threading.Thread(name=s, target=method_to_call)
                    d.setDaemon(True)
                    d.start()
                    self.l.info("Executed %s" % (str(s)))
                    res = "done"
                except Exception as e:
                    self.l.error("Error: " + str(e))
                    res = str(e)
            return srv.ConceptResponse(result=res)
        else:
            r = "Execution in process. Ignoring the request."
            self.l.info(r)
            return srv.ConceptResponse(result=r)

    def sensor_callback(self, data):
        self.l.debug("From SensorInterpreter: %s:0x%x" %
                    (data.perception_summary, data.argument))
        self.data[data.perception_summary] = data.argument
        self.l.debug(self.data)

    def start(self):
        self.l.debug("Node \'%s\' is starting..." % self.name)
        rospy.init_node(self.name, anonymous=False)

        self.publisher = com.get.publisher(topic_name=com.names.TOPIC_DEVICECMD,
                                           data_class=msg.DeviceCmd,
                                           logger=self.l)
        self.server = com.get.server(name=self.name,
                                     service=srv.Concept,
                                     handle=self.concept_handler,
                                     logger=self.l)
        self.subscriber = com.get.subscriber(
            topic_name=com.names.TOPIC_MAIN_SENSOR_INTERPRETER,
            data_class=msg.SensorConcept,
            callback=self.sensor_callback)

        self.l.info("[  DONE  ] Node \'%s\' is ready..." % self.name)
