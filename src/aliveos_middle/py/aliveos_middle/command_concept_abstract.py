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
from threading import Thread
from typing import TYPE_CHECKING, Union
import time

# to avoid circular import:
#   1. TYPE_CHECKING is always false on runtime
#   2. used strings in typing. See: https://www.python.org/dev/peps/pep-0484/#forward-references
if TYPE_CHECKING:
    from .concept2commands_interpreter import Concept2CommandsInterpreter


class CommandConceptAbstract:
    name = ""

    def __init__(self, c2c_obj: "Concept2CommandsInterpreter", mod: Union[tuple, str]):
        self._flag_continue = True
        self._flag_terminate = False
        self._c2c = c2c_obj
        self._period_sec = 0.01
        self._mod = []
        if isinstance(mod, tuple):
            self._mod = list(mod)
        elif isinstance(mod, str):
            # "('1',3,   'hello ')" -> ('1', '2', 'hello')
            half_raw_mod = (x.strip().strip("'").strip("\"") for x in mod.strip("(").strip(")").split(","))
            self._mod = list(half_raw_mod)
        else:
            raise TypeError
        self.thread = Thread(name=self.name, target=self.__execute_concept, daemon=True)

    def sleep(self, time_sec: float):
        remained_sec = time_sec
        while remained_sec > 0:
            # Pause and terminate
            while not self._flag_continue:
                time.sleep(self._period_sec)
                if self._flag_terminate:
                    return
            if self._flag_terminate:
                return
            # Step
            time.sleep(self._period_sec)
            remained_sec = remained_sec - self._period_sec

    def is_modifier(self, mod: str):
        return (mod in self._mod)

    def cmd(self, cmd: int, arg: int = 0, target: str = "all", message: str = ""):
        # Pause and terminate
        while not self._flag_continue:
            time.sleep(self._period_sec)
            if self._flag_terminate:
                return
        if self._flag_terminate:
            return
        # Action
        self._c2c.publish(cmd, arg, target, message)

    def pause_exec(self):
        self._flag_continue = False

    def continue_exec(self):
        self._flag_continue = True

    def stop_exec(self):
        self._flag_continue = False
        self._flag_terminate = True

    def start_exec_new_thread(self):
        self._flag_continue = True
        self._flag_terminate = False
        self.thread.start()

    def start_exec(self):
        self._flag_continue = True
        self._flag_terminate = False
        self.__execute_concept()

    def _main(self):
        """ Interaction with devices """
        raise NotImplementedError

    def _pre_main_analysis(self) -> bool:
        """ Adds modifiers if necessary nased on emotions, finds contradictions """
        return True

    def _post_main(self) -> bool:
        return True

    def __execute_concept(self):
        if self._pre_main_analysis():
            self._main()
            self._post_main()
        else:
            rospy.logerr("Failed modifiers check: %s", str(self._mod))
        self.stop_exec()

    def __call__(self):
        self.start_exec()
