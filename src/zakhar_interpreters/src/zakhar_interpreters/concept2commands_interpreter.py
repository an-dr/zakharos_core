from os.path import basename, splitext
from .concept2commands_interpreter_abstract import Concept2CommandsInterpreterAbstract
from .concepts import *
import rospy


class Concept2CommandsInterpreter(ConceptsBacicReactions, ConceptsPanic, ConceptsMove,
                                  Concept2CommandsInterpreterAbstract):
    def __init__(self):
        Concept2CommandsInterpreterAbstract.__init__(self, name=splitext(basename(__file__))[0])


def start():
    obj = Concept2CommandsInterpreter()
    obj.start()
    rospy.spin()
