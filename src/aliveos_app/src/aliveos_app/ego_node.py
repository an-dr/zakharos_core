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
from time import sleep
from .exceptions import ReceivedAbort, ReceivedBusy
from .generic_mind_node import GenericMindNode, node_types


class EgoNode(GenericMindNode):
    """
    This class has main function (shold be implemented) that starts with the node and works when it is not interrupted
    by other nodes
    """
    def __init__(self, name, concept_files: list = None):
        super().__init__(name=name, concept_files=concept_files, node_type=node_types.EGO_NODE)
        self.terminate_flag = False

    def _main(self):
        """
        Main function supervisor
        """
        while (1):
            if self.terminate_flag:
                break
            try:
                self.main()
            except ReceivedAbort:
                rospy.logwarn("Abort")
                continue

    def _to_will(self, symbol, modifier=()):
        rospy.loginfo("Ego is willing: %s:%s" % (symbol, str(modifier)))
        try:
            response = self.client_command_concept(self.node_type, symbol, str(modifier))
            res = response.result
            rospy.loginfo("Response for %s: %s" % (symbol, res))
            if res[:4] == "busy":
                raise ReceivedBusy
            elif res[:5] == "abort":
                raise ReceivedAbort
            elif res[:] == "error":
                rospy.logerr(f"Concept ({symbol}) was executed with {res}")
            return res
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            self.terminate_flag = True
            return str(e)

    def to_will(self, symbol, modifier=()):
        while 1:
            if self.terminate_flag:
                break
            try:
                return self._to_will(symbol, modifier)
            except ReceivedBusy:
                sleep(1)  # wait and retry

    def main(self):
        raise NotImplementedError

    def start(self):
        super().start()
        self._main()
