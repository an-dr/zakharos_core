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

import threading

from rosparam import get_param
from rospy.service import ServiceException
from rospy import logdebug, logerr, logwarn, logwarn_once, init_node, spin
from rospkg import RosPack, ResourceNotFound

import aliveos_py
from aliveos_py.helpers.json_tools import json_to_dict, dict_to_json_str, string_to_obj
from aliveos_msgs import srv, msg


class AbstractDevice():
    def __init__(
        self,
        dev_name: str,
        hw_server_name: str = None,
        emotion_core_data_descriptor_json: str = None,
        perception_concept_descriptor_json: str = None,
    ):
        """
        Parameters
        ----------
        dev_name : str
            [description]
        hw_server_name : str, optional
            If not specified, the HW server will not be used, by default None
        emotion_core_data_descriptor_json : str, optional
            If not specified, the data_descriptor server will not be used, by default None
        perception_concept_descriptor_json : str, optional
            If not specified, the perception_concept_descriptor server will not be used, by default None

        Raises
        ------
        ResourceNotFound
            [description]
        """

        self.name = dev_name
        self.hw_server_name = hw_server_name
        self.client_hw_lock = threading.Lock()

        try:
            self.schema_path = RosPack().get_path('aliveos_msgs') + "/json"
        except ResourceNotFound:
            raise ResourceNotFound("Cannot find the aliveos_msgs package")

        self.data_dsc_json = None
        if emotion_core_data_descriptor_json:
            self.data_dsc_json = json_to_dict(
                in_json=emotion_core_data_descriptor_json,
                in_schema=f"{self.schema_path}/emotion-core-data-descriptor.json")

        self.p_concept_json = None
        if perception_concept_descriptor_json:
            self.p_concept_json = json_to_dict(
                in_json=perception_concept_descriptor_json,
                in_schema=f"{self.schema_path}/perception-concept-descriptor.json")

        # ros clients
        self.client_hw = None
        self.client_emotion_core_data_descriptor = None
        self.client_perception_concept_decriptor = None
        # ros subscribers
        self.subscriber_to_cmd = None
        # ros publishers
        self.publisher_of_sensor_data = None

    def subscriber_to_cmd_callback(self, data: msg.DeviceCmd):
        if ((data.device == self.name) or (data.device == 'all')):
            logwarn(f"Got cmd: {data.cmd}({data.arg})")
            cmd_method = getattr(self, f"command_{data.cmd}", None)
            args = string_to_obj(data.arg)
            if cmd_method:
                cmd_method(args)
            else:
                self.command_unknown(arg_list=[{data.cmd}] + args)

    def command_unknown(self, arg_list):
        logerr(f"Unknown command {arg_list[0]}, args: {arg_list[1:]}")

    def publish_device_data(self,
                            data_value: float,
                            data_type: str = "value",
                            data_source: str = None) -> None:
        """
        Parameters
        ----------
        data : float
            [description]
        data_source : str, optional
            If not specified, the name of the device will be used, by default None
        data_type : str, optional
            [description], by default "value"
        """
        to_send = msg.DeviceData()
        if data_source:
            to_send.data_source = data_source
        else:
            to_send.data_source = self.name
        to_send.data_type = data_type
        to_send.data_value = data_value

        logdebug(f"{self.name} published: {to_send.data_source}-{to_send.data_type}-{to_send.data_value}")
        self.publisher_of_sensor_data.publish(to_send)

    def send_perception_concept_to_d2c(self):
        if self.client_perception_concept_decriptor:
            m = srv.PerceptionConceptDescriptorRequest()
            m.device_name = self.name
            m.descriptor_json = dict_to_json_str(self.p_concept_json)
            try:
                r = self.client_perception_concept_decriptor(m)
            except ServiceException:
                logerr("Service PerceptionConceptDescriptor error!")
                r = None
        else:
            logwarn_once("The function was turned off!")
            r = None
        return r

    def send_emotion_core_data_descriptor(self):
        if self.client_emotion_core_data_descriptor:
            m = srv.EmotionCoreDataDescriptorRequest()
            data_type = self.data_dsc_json["data_type"]
            m.sensor_name = f"{self.name}:{data_type}"
            m.val_max = self.data_dsc_json["value_max"]
            m.val_min = self.data_dsc_json["value_min"]
            m.weights_json = dict_to_json_str(self.data_dsc_json["weights"])
            try:
                r = self.client_emotion_core_data_descriptor(m)
            except ServiceException:
                logerr("Service EmotionCoreDataDescriptor error!")
                r = None
        else:
            logwarn_once("The function was turned off!")
            r = None
        return r

    def _init_communications(self):
        # Hardware server:
        if self.hw_server_name:
            self.client_hw = aliveos_py.ros.get.client(srv_name=self.hw_server_name, service=srv.Hw)

        # Emotion Core:
        if self.data_dsc_json:
            self.client_emotion_core_data_descriptor = aliveos_py.ros.get.client(
                srv_name=get_param("SRV_ECORE_DDSC"), service=srv.EmotionCoreDataDescriptor)
            self.send_emotion_core_data_descriptor()

        # D2C:
        if self.p_concept_json:
            self.client_perception_concept_decriptor = aliveos_py.ros.get.client(
                srv_name=get_param("SRV_D2C_PCDSC"), service=srv.PerceptionConceptDescriptor)
            self.send_perception_concept_to_d2c()

        # C2C:
        self.subscriber_to_cmd = aliveos_py.ros.get.subscriber(topic_name=get_param("TOPIC_DEV_CMD"),
                                                               data_class=msg.DeviceCmd,
                                                               callback=self.subscriber_to_cmd_callback)
        # Device itself
        self.publisher_of_sensor_data = aliveos_py.ros.get.publisher(topic_name=get_param("TOPIC_DEV_DATA"),
                                                                     data_class=msg.DeviceData)

    def _run(self) -> None:
        init_node(self.name, anonymous=False)
        self._init_communications()
        self.main()

    def start(self):
        self._run()
        spin()

    def main(self):
        raise NotImplementedError
