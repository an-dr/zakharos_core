import rospy
import threading
import zakhar_common as com
from zakhar_msgs import msg, srv
from os.path import splitext, basename
from typing import Union, List, Any
from .command_concept_abstract import CommandConceptAbstract
from .concepts import Move, Avoid, BasicPanic, Express, Shiver


class Concept2CommandsInterpreter:
    def __init__(self, name="Concept2CommandsInterpreter"):
        self.name = name
        self.publisher_to_devices = None
        self.subscriber_to_perception_concepts = None
        self.server_of_command_concepts = None  # type: Union[rospy.Service, None]
        self.exec_in_progress = False
        self.data = {}
        self.concepts = []  # type: List[CommandConceptAbstract]

    def publish(self, cmd: int, arg: int = 0, target: str = "all", message: str = ""):
        to_send = msg.DeviceCmd()
        to_send.target = target
        to_send.cmd = cmd
        to_send.arg = arg
        to_send.message = message
        self.publisher_to_devices.publish(to_send)
        rospy.logdebug("Send to %s: 0x%x(0x%x). [%s]" % (target, cmd, arg, message))

    def add_concept(self, cc: CommandConceptAbstract):
        # type check
        if not isinstance(cc, type) and cc.__name__ == CommandConceptAbstract.__name__:
            raise TypeError("Type of cc is %s" % str(type(cc)))

        # check uniqueness
        for i in self.concepts:
            if i.name == cc.name:
                raise ValueError("A concept with the same name is already present")

        self.concepts.append(cc)

    def command_concept_handler(self, req: srv.CommandConceptRequest) -> srv.CommandConceptResponse:
        concept = req.symbol
        raw_mod = req.modifier
        half_raw_mod = (x.strip().strip("'").strip("\"") for x in raw_mod.strip("(").strip(")").split(","))
        mod = tuple(half_raw_mod)

        ego_type = req.ego_type
        rospy.loginfo("Got %s with source type: %d" % (concept, ego_type))

        if not self.exec_in_progress:
            self.exec_in_progress = True
            rospy.logdebug("Object: %s; Method: %s" % (str(self), str(concept)))

            to_call = None  # type: Any[None, CommandConceptAbstract]
            for i in self.concepts:
                if i.name == concept:
                    to_call = i
            if to_call is None:
                rospy.logerr("Unknown concept")
                res = "no concept"
            else:
                try:
                    rospy.logdebug("Executing...")
                    d = threading.Thread(name=concept, target=to_call.execute, kwargs={"c2c": self, "modifier": mod})
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
        rospy.logdebug("From SensorInterpreter: %s:%s" % (data.symbol, data.modifier))
        self.data[data.symbol] = data.modifier
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


def start():
    obj = Concept2CommandsInterpreter(name=splitext(basename(__file__))[0])
    obj.add_concept(Move)
    obj.add_concept(Avoid)
    obj.add_concept(Express)
    obj.add_concept(Shiver)
    obj.add_concept(BasicPanic)
    obj.start()
    rospy.spin()
