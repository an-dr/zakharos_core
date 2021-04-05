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

from .generic_mind_node import GenericMindNode, node_types
from aliveos_msgs import msg
import rospy


class InstinctNode(GenericMindNode):
    def __init__(self, name, concept_files):
        super().__init__(name=name, concept_files=concept_files, node_type=node_types.INSTINCT_NODE)

    def start(self):
        super().start()
        rospy.spin()

    def _perception_callback(self, perception_concept: msg.PerceptionConcept):
        super()._perception_callback(perception_concept)
        self.perceprion_callback(perception_concept.symbol, perception_concept.modifier)

    def perceprion_callback(self, symbol, modifier):
        raise NotImplementedError
