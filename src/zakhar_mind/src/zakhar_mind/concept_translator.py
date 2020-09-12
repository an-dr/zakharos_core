import rospy
from os.path import basename, splitext
import zakhar_common as com
from zakhar_pycore import zakhar__log as log
import zakhar_i2c_devices as dev
from zakhar_msgs import msg, srv
from .concept_translator_abstract import ConceptTranslatorAbstract
from .concepts import *


class ConceptTranslator(ConceptsBacicReactions,
                        ConceptsPanic,
                        ConceptsMove,
                        ConceptTranslatorAbstract):
    def __init__(self, name="ConceptTranslator", log_level=log.INFO):
        ConceptTranslatorAbstract.__init__(self,
                                           name=name,
                                           log_level=log_level)

FNAME = splitext(basename(__file__))[0]  # filename without extension
concept_translator = ConceptTranslator(name=FNAME, log_level=log.INFO)