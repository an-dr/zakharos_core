#!/usr/bin/env python3
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
from aliveos_hw.abstract_device import AbstractDevice
import os
from time import sleep
from random import randrange


class SimpleDevice(AbstractDevice):
    def main(self):
        while (1):
            self.publish_device_data(data_value=randrange(-100, 100, 1))
            sleep(1)


d = SimpleDevice(
    dev_name="simple_device",
    #  hw_server_name="simple_hw_server",
    hw_server_name=None,
    perception_concept_descriptor_json=f"{os.path.dirname(__file__)}/" + "simple_device _pc_dsc.json",
    emotion_core_data_descriptor_json="""
{
    "$schema": "perception-concept-descriptor.json",
    "data_type": "value",
    "value_min": 0,
    "value_max": 100,
    "weights": [
        { "parameter": "adrenaline", "value": -0.1 },
        { "parameter": "dopamine", "value": 2 }
    ]
}
    """)
d.start()
