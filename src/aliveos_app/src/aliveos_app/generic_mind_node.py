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
import json
from aliveos_msgs import srv, msg
from aliveos_py import ros as ar
from aliveos_py.helpers.json_tools import json_to_dict, dict_to_json_str, ValidationError
from rospy.service import ServiceException
from rospkg import RosPack, ResourceNotFound
from . import node_types


class GenericMindNode:
    def __init__(self, name="GenericMindNode", concept_files: list = None, node_type=node_types.GENERIC_NODE):
        self.name = name
        self.node_type = node_type
        self.sensor_cb_thread = None
        self.current_emotion_params = None
        self.current_perception_concept = None
        self.current_perception_concept_mod = None
        # Command concepts
        self.concept_files = concept_files
        # Clients
        self.client_command_concept = None
        self.client_command_concept_dsc = None
        self.client_emotion_core_write = None
        # Subscribers
        self.subscriber_perception_concept = None
        self.subscriber_emotion_params = None

        try:
            self.schema_path = RosPack().get_path('aliveos_msgs') + "/json"
        except ResourceNotFound:
            raise ResourceNotFound("Cannot find the aliveos_msgs package")

    def _perception_callback(self, perception_concept: msg.PerceptionConcept):
        self.current_perception_concept = perception_concept.symbol
        self.current_perception_concept_mod = perception_concept.modifier

    def _emotion_callback(self, params: msg.EmotionParams):
        params_dict = json.loads(params.params_json)
        self.current_emotion_params = params_dict

    def _start_communications(self):
        rospy.logdebug("Client \'%s\' is starting..." % self.name)
        self.subscriber_perception_concept = ar.get.subscriber(topic_name=ar.topics.MAIN_SENSOR_INTERPRETER,
                                                               data_class=msg.PerceptionConcept,
                                                               callback=self._perception_callback)
        self.subscriber_emotion_params = ar.get.subscriber(topic_name=ar.topics.EMOTION_PARAMS,
                                                           data_class=msg.EmotionParams,
                                                           callback=self._emotion_callback)
        self.client_command_concept = rospy.ServiceProxy(ar.services.CONCEPT_TO_COMMAND, srv.CommandConcept)
        self.client_of_emotion_core_write = ar.get.client(srv_name=ar.services.EMOTIONCORE_WRITE,
                                                          service=srv.EmotionCoreWrite)
        self.client_command_concept_dsc = ar.get.client(srv_name=ar.services.C2C_COMMAND_CONCEPT_DSC,
                                                        service=srv.CommandConceptDescriptor)
        rospy.loginfo("Client \'%s\' is ready" % self.name)

    def _send_command_concept_single(self, cc):
        json_dict = json_to_dict(in_json=cc, in_schema=f"{self.schema_path}/command-concept-dsc.json")
        json_str = dict_to_json_str(in_dict=json_dict)
        m = srv.CommandConceptDescriptorRequest()
        m.descriptor_json = json_str
        self.client_command_concept_dsc(m)

    def _send_command_concepts(self):
        for cc in self.concept_files:
            try:
                self._send_command_concept_single(cc)
            except ValidationError as e:
                rospy.logerr(f"Incorrect input json: {cc}\nError:[{e.message}]")

    def start(self):
        rospy.logdebug("Node \'%s\' is starting..." % self.name)
        rospy.init_node(self.name, anonymous=False)
        self._start_communications()
        self._send_command_concepts()
        rospy.loginfo("[  DONE  ] Node \'%s\' is ready..." % self.name)

    def write_to_emotion_core(self, value: int, change_per_sec: int,
                              param: str) -> srv.EmotionCoreWriteResponse:
        m = srv.EmotionCoreWriteRequest()
        m.value = value
        m.temp_val_per_sec = change_per_sec
        m.temp_param_name = param
        try:
            r = self.client_of_emotion_core_write(m)
        except ServiceException:
            rospy.logerr("Service PerceptionConceptDescriptor error!")
            r = None
        return r

    def __call__(self):
        self.start()
