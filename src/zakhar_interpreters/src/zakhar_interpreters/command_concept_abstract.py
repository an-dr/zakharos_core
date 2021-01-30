import rospy
from typing import TYPE_CHECKING

# to avoid circular import:
#   1. TYPE_CHECKING is always false on runtime
#   2. used strings in typing. See: https://www.python.org/dev/peps/pep-0484/#forward-references
if TYPE_CHECKING:
    from .concept2commands_interpreter import Concept2CommandsInterpreter


class CommandConceptAbstract:
    name = ""

    @classmethod
    def action(cls, c2c: "Concept2CommandsInterpreter", modifier: list = []):
        """ Interaction with devices """
        raise NotImplementedError

    @classmethod
    def modifier_analysis(cls, modifier: list = [], emotion_params: dict = {}) -> bool:
        """ Adds modifiers if necessary nased on emotions, finds contradictions """
        raise NotImplementedError

    @classmethod
    def __call__(cls, c2c: "Concept2CommandsInterpreter", modifier: dict = (), emotion_params: dict = {}):
        mod_list = list(modifier)  # to make it mutable
        if cls.modifier_analysis(cls, mod_list, emotion_params):
            c2c.exec_in_progress = True  # FIX: probably duplicates somethig
            try:
                cls.action(cls, c2c=c2c, modifier=mod_list)
            except Exception as e:
                rospy.logerr(str(e))
            c2c.exec_in_progress = False
        else:
            rospy.logerr("Failed modifiers check: %s", str(mod_list))

    @classmethod
    def execute(cls, c2c: "Concept2CommandsInterpreter", modifier: dict = (), emotion_params: dict = {}):
        cls.__call__(c2c=c2c, modifier=modifier, emotion_params=emotion_params)
