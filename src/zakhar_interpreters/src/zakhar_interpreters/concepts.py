import rospy
from time import sleep
from .command_concept_abstract import CommandConceptAbstract
from zakhar_pycore.constants import CMD
from typing import TYPE_CHECKING

# to avoid circular import:
#   1. TYPE_CHECKING is always false on runtime
#   2. used strings in typing. See: https://www.python.org/dev/peps/pep-0484/#forward-references
if TYPE_CHECKING:
    from .concept2commands_interpreter import Concept2CommandsInterpreter


class Move(CommandConceptAbstract):
    name = "move"

    class modifiers:
        forward = "forward"
        backward = "backward"
        left = "left"
        right = "right"
        slower = "slower"
        slow = "slow"
        stop = "stop"

    def modifier_analysis(cls, modifier: list = [], emotion_params: dict = {}) -> bool:
        """ Adds modifiers if necessary nased on emotions, finds contradictions """
        return True

    def action(cls, c2c: "Concept2CommandsInterpreter", modifier: list = [], emotion_params: dict = {}):
        """ Interaction with devices """
        rospy.loginfo("move %s" % str(modifier))
        # speed
        if cls.modifiers.slower in modifier:
            c2c.publish(target="moving_platform", cmd=CMD.MOVE.SPEED2)
        if cls.modifiers.slow in modifier:
            c2c.publish(target="moving_platform", cmd=CMD.MOVE.SPEED1)

        for direction in modifier:
            if direction == cls.modifiers.forward:
                c2c.publish(target="moving_platform", cmd=CMD.MOVE.FORWARD)
            if direction == cls.modifiers.backward:
                c2c.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
            if direction == cls.modifiers.left:
                c2c.publish(target="moving_platform", cmd=CMD.MOVE.LEFT)
            if direction == cls.modifiers.right:
                c2c.publish(target="moving_platform", cmd=CMD.MOVE.RIGHT)
            if direction == cls.modifiers.stop:
                c2c.publish(target="moving_platform", cmd=CMD.MOVE.STOP)

        # restore speed
        if (cls.modifiers.slower in modifier) or (cls.modifiers.slow in modifier):
            c2c.publish(target="moving_platform", cmd=CMD.MOVE.SPEED3)



class Shiver(CommandConceptAbstract):
    name = "shiver"

    def action(cls, c2c: "Concept2CommandsInterpreter", modifier: list = [], emotion_params: dict = {}):
        """ Interaction with devices """
        c2c.publish(target="moving_platform", cmd=CMD.MOVE.SHIVER)


    def modifier_analysis(cls, modifier: list = [], emotion_params: dict = {}) -> bool:
        """ Adds modifiers if necessary nased on emotions, finds contradictions """
        return True



class Avoid(CommandConceptAbstract):
    name = "avoid"

    class modifiers:
        front = "front"
        left = "left"
        right = "right"

    def modifier_analysis(cls, modifier: list = [], emotion_params: dict = {}) -> bool:
        """ Adds modifiers if necessary nased on emotions, finds contradictions """
        if (("left" in modifier) or ("right" in modifier)) and not ("front" in modifier):
            modifier.append("front")

        if ("left" in modifier) and ("right" in modifier):
            modifier.remove("left")
            modifier.remove("right")
        return True

    def action(cls, c2c: "Concept2CommandsInterpreter", modifier: list = [], emotion_params: dict = {}):
        """ Interaction with devices """
        if cls.modifiers.front in modifier:
            c2c.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
            sleep(.2)
            c2c.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
            sleep(.2)
            c2c.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
            sleep(.2)
        if cls.modifiers.left in modifier:
            c2c.publish(target="moving_platform", cmd=CMD.MOVE.RIGHT, arg=60)
            sleep(1)
        if cls.modifiers.right in modifier:
            c2c.publish(target="moving_platform", cmd=CMD.MOVE.LEFT, arg=60)
            sleep(1)



class BasicPanic(CommandConceptAbstract):
    name = "basic_panic"

    def modifier_analysis(cls, modifier: list = [], emotion_params: dict = {}) -> bool:
        """ Adds modifiers if necessary nased on emotions, finds contradictions """
        return True

    def action(cls, c2c: "Concept2CommandsInterpreter", modifier: list = [], emotion_params: dict = {}):
        """ Interaction with devices """
        rospy.loginfo("basic_panic")
        c2c.publish(target="face_platform", cmd=CMD.FACE.SAD)

        panic_light = int(c2c.data.get("light", 0))
        current_light = panic_light
        rospy.loginfo("panic_light : 0x%x" % panic_light)
        c2c.publish(target="moving_platform", cmd=CMD.MOVE.SHIVER)
        sleep(2)
        while not (current_light > 1.2 * panic_light):
            c2c.publish(target="moving_platform", cmd=CMD.MOVE.FORWARD)
            sleep(.05)
            current_light = int(c2c.data.get("light", 0xFFFFFFFF))
            rospy.loginfo("current_light : 0x%x" % current_light)
        rospy.loginfo("out of loop")
        sleep(.5)
        c2c.publish(target="moving_platform", cmd=CMD.MOVE.STOP)
        sleep(.5)
        c2c.publish(target="moving_platform", cmd=CMD.MOVE.SHIVER)
        c2c.publish(target="face_platform", cmd=CMD.FACE.CALM)



class Express(CommandConceptAbstract):
    name = "express"

    class modifiers:
        hello = "hello"
        bye = "bye"

    def action(cls, c2c: "Concept2CommandsInterpreter", modifier: list = [], emotion_params: dict = {}):
        """ Interaction with devices """
        if cls.modifiers.hello in modifier:
            c2c.publish(target="face_platform", cmd=CMD.FACE.CALM)
        elif cls.modifiers.bye in modifier:
            c2c.publish(target="face_platform", cmd=CMD.FACE.BLINK)


    def modifier_analysis(cls, modifier: list = [], emotion_params: dict = {}) -> bool:
        """ Adds modifiers if necessary nased on emotions, finds contradictions """
        return True
