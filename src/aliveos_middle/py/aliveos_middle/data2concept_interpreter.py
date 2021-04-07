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

from rosparam import get_param
from rospy import logdebug, logwarn, init_node, ServiceException, spin

from aliveos_msgs import msg, srv
from aliveos_py import ros as ar
from aliveos_py.helpers.json_tools import json_to_dict


class Data2ConceptInterpreter:
    def __init__(self):
        self.current_emotion_params = None
        self.current_device_data = {}
        self.perception_concepts = {}
        # Publishers:
        self.publisher_to_egos = None
        # Subscribers:
        self.subscriber_to_devices = None
        self.subscriber_to_emotion_params = None
        # Clients:
        self.client_of_emotion_core_write = None
        self.client_of_emotion_data_descriptor = None
        # Servers
        self.server_of_perception_concept_dsc = None

    def publish_perception_concept_to_egos(self, symbol, modifier):
        to_send = msg.PerceptionConcept()
        to_send.symbol = symbol
        to_send.modifier = modifier
        self.publisher_to_egos.publish(to_send)
        logdebug("Sent [%s, %s]" % (symbol, modifier))

    def _check_condition(self, device_data_name: str, condition: str, threshold: float) -> bool:
        device_data_val = self.current_device_data.get(device_data_name)
        if not device_data_val:
            return False
        if condition == "EQUAL":
            return device_data_val == threshold
        elif condition == "NOT_EQUALS":
            return device_data_val != threshold
        elif condition == "GREATER_THAN":
            return device_data_val > threshold
        elif condition == "GREATER_THAN_OR_EQUAL":
            return device_data_val >= threshold
        elif condition == "LESS_THAN":
            return device_data_val < threshold
        elif condition == "LESS_THAN_OR_EQUAL":
            return device_data_val <= threshold
        else:
            return False

    def device_data_handler(self, data: msg.DeviceData):
        dev_name = f"{data.data_source}:{data.data_type}"
        logdebug(f"Received from {dev_name} - {data.data_value}")
        # Emotion Core
        m = srv.EmotionCoreWriteRequest()
        m.sensor_name = dev_name
        m.value = int(data.data_value)  # TODO: make it float
        self.current_device_data[f"{data.data_source}:{data.data_type}"] = data.data_value
        try:
            self.client_of_emotion_core_write(m)
        except ServiceException:
            logwarn("Service %s did not process request. Is it running?" % ar.services.EMOTIONCORE_WRITE)

        # Data analysis
        dev = self.perception_concepts.get(dev_name)
        for entry in dev:
            r = False
            concept = entry["concept"]
            conditions = entry["conditions"]
            for cond in conditions:
                r = self._check_condition(device_data_name=dev_name,
                                          condition=cond["condition"],
                                          threshold=cond["threshold"])
                if not r:
                    break
            if r:
                logdebug("Concept is: %s" % concept)
                self.publish_perception_concept_to_egos(symbol=concept, modifier="")

    def handler_perception_concetps_dsc(self, req: srv.PerceptionConceptDescriptorRequest) -> \
            srv.PerceptionConceptDescriptorResponse:
        j_dict = json_to_dict(req.descriptor_json)
        for d_type in j_dict["data_types"]:
            dev = req.device_name + ":" + d_type["data_type"]
            self.perception_concepts[dev] = d_type["descriptor"]
        return srv.PerceptionConceptDescriptorResponse(result="ok")

    def handler_emotion_params(self, data: msg.EmotionParams) -> None:
        self.current_emotion_params = json_to_dict(data.params_json)

    def init_communications(self):
        self.publisher_to_egos = ar.get.publisher(topic_name=get_param("TOPIC_PC"),
                                                  data_class=msg.PerceptionConcept)
        self.subscriber_to_devices = ar.get.subscriber(topic_name=get_param("TOPIC_DEV_DATA"),
                                                       data_class=msg.DeviceData,
                                                       callback=self.device_data_handler)
        self.subscriber_to_emotion_params = ar.get.subscriber(topic_name=get_param("TOPIC_EPARAM"),
                                                              data_class=msg.EmotionParams,
                                                              callback=self.handler_emotion_params)
        self.client_of_emotion_core_write = ar.get.client(srv_name=get_param("SRV_ECORE_W"),
                                                          service=srv.EmotionCoreWrite)
        self.client_of_emotion_data_descriptor = ar.get.client(srv_name=get_param("SRV_ECORE_DDSC"),
                                                               service=srv.EmotionCoreDataDescriptor)
        self.server_of_perception_concept_dsc = ar.get.server(name=get_param("SRV_D2C_PCDSC"),
                                                              service=srv.PerceptionConceptDescriptor,
                                                              handle=self.handler_perception_concetps_dsc)

    def start(self):
        init_node(name=self.__class__.__name__, anonymous=False)
        self.init_communications()


def start():
    obj = Data2ConceptInterpreter()
    obj.start()
    spin()
