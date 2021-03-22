import rospy
from zakhar_pycore import ros as zros
from zakhar_msgs import msg, srv
from os.path import splitext, basename
from typing import Union, List
from zakhar_mind.mind_classes import node_types
from .command_concept_abstract import CommandConceptAbstract
from .concepts import Move, Avoid, BasicPanic, Express, Shiver
from threading import Lock
from uuid import uuid1


class Concept2CommandsInterpreter:
    RESPONSE_ERROR = "error"
    RESPONSE_OK = "ok"
    RESPONSE_BUSY = "busy"
    RESPONSE_ABORT = "abort"

    CODE_ERROR = -1
    CODE_OK = 0
    CODE_BUSY = 1
    CODE_ABORT = 2

    def __init__(self, name="Concept2CommandsInterpreter"):
        self.name = name
        self.publisher_to_devices = None
        self.subscriber_to_perception_concepts = None
        self.server_of_command_concepts = None  # type: Union[rospy.Service, None]
        self.lock_consciousness = Lock()
        self.lock_consciousness_pause = Lock()
        self.lock_consciousness_abort = Lock()
        self.lock_instinct = Lock()
        self.executing_reflex_concepts = []
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

    def get_concept(self, concept: str) -> Union[CommandConceptAbstract, None]:
        for i in self.concepts:
            if i.name == concept:
                return i
        rospy.logerr("Unknown concept")
        return None

    def exec_concept(self, concept_class: type, modifier: str) -> str:
        obj = concept_class(c2c_obj=self, mod=modifier)  # type: CommandConceptAbstract
        try:
            rospy.logdebug("Executing %s..." % obj.name)
            obj.start_exec()
            rospy.loginfo("Executed %s" % (str(obj.name)))
            return self.RESPONSE_OK
        except Exception as e:
            rospy.logerr("Error: " + str(e))
            return ("%s: %s" % (self.RESPONSE_ERROR, str(e)))

    # TODO split into is_permisson and apply_permission
    def take_permission(self, mind_node_type: int, concept: str) -> int:
        """
        Parameters
        ----------
        mind_node_type : int
        concept : str

        Returns
        -------
        int
            code (see self.CODE_*)
        """
        result = self.CODE_BUSY

        if mind_node_type == node_types.EGO_NODE:
            if self.lock_consciousness_abort.locked():
                result = self.CODE_ABORT
            elif self.lock_consciousness.acquire(blocking=False) and not self.lock_consciousness_pause.locked():
                # rospy.logwarn(f"Acquired by {concept} (node type: {mind_node_type})")
                result = self.CODE_OK
        elif mind_node_type == node_types.INSTINCT_NODE:
            self.lock_consciousness_abort.acquire(blocking=False)  # will be unlocked when the abort be sent
            if self.lock_instinct.acquire(blocking=False):
                # rospy.logwarn(f"Acquired by {concept} (node type: {mind_node_type})")
                result = self.CODE_OK
        elif mind_node_type == node_types.REFLEX_NODE:
            if concept in self.executing_reflex_concepts:  # avoiding executing the same reflex twice
                rospy.loginfo(f"Reflex busy: {concept} in  {self.executing_reflex_concepts}")
                result = self.CODE_BUSY
            else:
                rospy.loginfo("Reflex added: " + str(concept))
                self.executing_reflex_concepts.append(concept)
                self.lock_consciousness_pause.acquire(blocking=False)
                rospy.logwarn(f"Acquired by {concept} (node type: {mind_node_type})")
                result = self.CODE_OK
        # rospy.logwarn(f"Locks: c - {self.lock_consciousness.locked()},  c_pause - {self.lock_consciousness_pause.locked()}, c_abort - {self.lock_consciousness_abort.locked()}, i - {self.lock_instinct.locked()}")
        rospy.logdebug("Permission is taken with code: %d (node_type: %d)" % (result, mind_node_type))
        return result

    def release_permissions(self, mind_node_type: int, concept: str):
        # rospy.logwarn(f"Released by {concept} (node type: {mind_node_type})")
        if mind_node_type == node_types.EGO_NODE:
            self.lock_consciousness.release()
        elif mind_node_type == node_types.INSTINCT_NODE:
            if self.lock_consciousness.locked():
                self.lock_consciousness.release()
            self.lock_instinct.release()
        elif mind_node_type == node_types.REFLEX_NODE:
            self.executing_reflex_concepts.remove(concept)
            rospy.loginfo(f"Reflex {concept} ended. Executing now: {self.executing_reflex_concepts}")
            if self.lock_consciousness.locked():
                self.lock_consciousness.release()
            self.lock_consciousness_pause.release()

    def command_concept_handler(self, req: srv.CommandConceptRequest) -> srv.CommandConceptResponse:
        res = ""
        concept = req.symbol
        mods = req.modifier
        node_type = req.ego_type
        rospy.loginfo("Got %s, mods: %s, type: %d" % (concept, mods, node_type))

        permission = self.take_permission(node_type, concept)

        if permission == self.CODE_OK:
            rospy.logdebug("Object: %s; Method: %s" % (str(self), str(concept)))
            conc_class = self.get_concept(concept)
            if conc_class is None:
                res = self.RESPONSE_ERROR
            else:
                res = self.exec_concept(conc_class, mods)
            self.release_permissions(node_type, concept)
        elif permission == self.CODE_BUSY:
            res = f"{self.RESPONSE_BUSY} ({node_type}, {concept}, {mods}) : Ignoring the request."
            rospy.logwarn(res)
        elif permission == self.CODE_ABORT:
            self.lock_consciousness_abort.release()
            res = self.RESPONSE_ABORT
        else:
            res = self.RESPONSE_ERROR

        return srv.CommandConceptResponse(result=res)

    def sensor_callback(self, data: msg.PerceptionConcept):
        rospy.logdebug("From SensorInterpreter: %s:%s" % (data.symbol, data.modifier))
        self.data[data.symbol] = data.modifier
        rospy.logdebug(self.data)

    def start(self):
        rospy.logdebug("Node \'%s\' is starting..." % self.name)
        rospy.init_node(self.name, anonymous=False)

        self.publisher_to_devices = zros.get.publisher(topic_name=zros.topics.DEVICECMD, data_class=msg.DeviceCmd)
        self.server_of_command_concepts = zros.get.server(name=self.name,
                                                          service=srv.CommandConcept,
                                                          handle=self.command_concept_handler)
        self.subscriber_to_perception_concepts = zros.get.subscriber(topic_name=zros.topics.MAIN_SENSOR_INTERPRETER,
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
