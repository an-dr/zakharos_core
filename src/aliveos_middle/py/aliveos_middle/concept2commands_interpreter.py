# *************************************************************************
#
# Copyright (c) 2021 Andrei Gramakov. All rights reserved.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# site:    https://agramakov.me
# e-mail:  mail@agramakov.me
#
# *************************************************************************

import rospy
from aliveos_py import ros as ar
from aliveos_msgs import msg, srv
from os.path import splitext, basename
from typing import Union, List
from aliveos_app import node_types
from .command_concept_abstract import CommandConceptAbstract
from aliveos_py.helpers.json_tools import json_to_dict
from threading import Lock
# from uuid import uuid1


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
        self.lock_consciousness = Lock()
        self.lock_consciousness_pause = Lock()
        self.lock_consciousness_abort = Lock()
        self.lock_instinct = Lock()
        self.executing_reflex_concepts = []
        self.data = {}
        self.concepts = {}

        # Servers
        self.server_of_command_concepts = None  # type: Union[rospy.Service, None]
        self.server_of_command_concept_descriptors = None  # type: Union[rospy.Service, None]
        # Publishers
        self.publisher_to_devices = None

        # Subscribers
        self.subscriber_to_perception_concepts = None

    def publish_device_cmd(self, device: str, cmd: str, arg: str):
        to_send = msg.DeviceCmd()
        to_send.device = device
        to_send.cmd = cmd
        to_send.arg = arg
        self.publisher_to_devices.publish(to_send)
        rospy.logdebug(f"Send to {device}: {cmd}({arg})")

    def add_concept(self, cc: CommandConceptAbstract):
        # type check
        if not isinstance(cc, type) and cc.__name__ == CommandConceptAbstract.__name__:
            raise TypeError("Type of cc is %s" % str(type(cc)))

        # check uniqueness
        for i in self.concepts:
            if i.name == cc.name:
                raise ValueError("A concept with the same name is already present")

        self.concepts.append(cc)

    def get_concept(self, concept: str) -> Union[list, None]:
        c = self.concepts.get(concept)
        if c:
            return c
        rospy.logerr(f"Unknown concept: {concept}")
        return None

    def exec_concept(self, concept_dsc: list, modifier: str) -> str:
        commands = None
        if modifier == "()":
            modifier = ""
        for c in concept_dsc:
            if c["modifier"] is modifier:
                commands = c["commands"]
                break
        if not commands:
            msg = f"There is no such modifier: {modifier}"
            rospy.logerr(msg)
            return f"Error: {msg}"

        for cmd in commands:
            self.publish_device_cmd(device=cmd["device_name"],
                                    cmd=cmd["command"],
                                    arg=str(cmd.get("arguments")))
        return self.RESPONSE_OK

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
            elif self.lock_consciousness.acquire(
                    blocking=False) and not self.lock_consciousness_pause.locked():
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
        # rospy.logwarn(f"Locks: c - {self.lock_consciousness.locked()},  c_pause -
        # {self.lock_consciousness_pause.locked()}, c_abort - {self.lock_consciousness_abort.locked()},
        # i - {self.lock_instinct.locked()}")
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

    def handler_command_concept(self, req: srv.CommandConceptRequest) -> srv.CommandConceptResponse:
        res = ""
        concept = req.symbol
        mods = req.modifier
        node_type = req.ego_type
        rospy.loginfo("Got %s, mods: %s, type: %d" % (concept, mods, node_type))

        permission = self.take_permission(node_type, concept)

        if permission == self.CODE_OK:
            rospy.logdebug("Object: %s; Method: %s" % (str(self), str(concept)))
            conc_dsc = self.get_concept(concept)
            if conc_dsc is None:
                res = self.RESPONSE_ERROR
            else:
                res = self.exec_concept(conc_dsc, mods)
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

    def handler_command_concept_descriptor(
            self, req: srv.CommandConceptDescriptorRequest) -> srv.CommandConceptDescriptorResponse:
        rospy.logdebug("handler_command_concept_descriptor: %s" % req.descriptor_json)
        json_dict = json_to_dict(req.descriptor_json)

        name = json_dict["name"]
        dsc = json_dict["descriptor"]
        if self.concepts.get(name):
            rospy.logerr(f"Command concept {name} already exists!")
            return srv.CommandConceptDescriptorResponse(result=self.RESPONSE_ERROR)
        self.concepts[name] = dsc
        return srv.CommandConceptDescriptorResponse(result=self.RESPONSE_OK)

    def sensor_callback(self, data: msg.PerceptionConcept):
        rospy.logdebug("From SensorInterpreter: %s:%s" % (data.symbol, data.modifier))
        self.data[data.symbol] = data.modifier
        rospy.logdebug(self.data)

    def start(self):
        rospy.logdebug("Node \'%s\' is starting..." % self.name)
        rospy.init_node(self.name, anonymous=False)

        self.publisher_to_devices = ar.get.publisher(topic_name=ar.topics.DEVICECMD, data_class=msg.DeviceCmd)
        self.server_of_command_concepts = ar.get.server(name=self.name,
                                                        service=srv.CommandConcept,
                                                        handle=self.handler_command_concept)
        self.server_of_command_concept_descriptors = ar.get.server(
            name=ar.services.C2C_COMMAND_CONCEPT_DSC,
            service=srv.CommandConceptDescriptor,
            handle=self.handler_command_concept_descriptor)

        self.subscriber_to_perception_concepts = ar.get.subscriber(
            topic_name=ar.topics.MAIN_SENSOR_INTERPRETER,
            data_class=msg.PerceptionConcept,
            callback=self.sensor_callback)

        rospy.loginfo("[  DONE  ] Node \'%s\' is ready..." % self.name)


def start():
    obj = Concept2CommandsInterpreter(name=splitext(basename(__file__))[0])
    obj.start()
    rospy.spin()
