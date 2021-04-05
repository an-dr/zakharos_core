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

from time import sleep
from aliveos_app import EgoNode
from os.path import dirname


class EgoSimple(EgoNode):
    def main(self):
        n = 0
        while (1):
            self.to_will("something")
            n += 1
            if n % 5:
                self.write_to_emotion_core(param="serotonin", value=100, change_per_sec=20)
            sleep(1)


node = EgoSimple("ego_simple", concept_files=[f"{dirname(__file__)}/simple_ego_cc_dsc.json"])
node()
