import rospy
from time import sleep
from .concept2commands_interpreter_abstract import Concept2CommandsInterpreterAbstract
from .command_concept_abstract import CommandConceptAbstract
from zakhar_pycore.constants import CMD
from typing import Any


class Move(CommandConceptAbstract):
    name = "move"

    def action(self, c2c: Concept2CommandsInterpreterAbstract, modifier: list = [], emotion_params: dict = {}):
        """ Interaction with devices """
        rospy.loginfo("move %s" % str(modifier))
        # speed
        if "slower" in modifier:
            c2c.publish(target="moving_platform", cmd=CMD.MOVE.SPEED2)
        if "slow" in modifier:
            c2c.publish(target="moving_platform", cmd=CMD.MOVE.SPEED1)

        for direction in modifier:
            if direction == "forward":
                c2c.publish(target="moving_platform", cmd=CMD.MOVE.FORWARD)
            if direction == "backward":
                c2c.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
            if direction == "left":
                c2c.publish(target="moving_platform", cmd=CMD.MOVE.LEFT)
            if direction == "right":
                c2c.publish(target="moving_platform", cmd=CMD.MOVE.RIGHT)
            if direction == "stop":
                c2c.publish(target="moving_platform", cmd=CMD.MOVE.STOP)

        # restore speed
        if ("slower" in modifier) or ("slow" in modifier):
            self.publish(target="moving_platform", cmd=CMD.MOVE.SPEED3)

        c2c.exec_in_progress = False

    def modifier_analysis(cls, modifier: list = [], emotion_params: dict = {}) -> bool:
        """ Adds modifiers if necessary nased on emotions, finds contradictions """
        return True


class ConceptsMove(Concept2CommandsInterpreterAbstract):
    def move_shiver(self, modifier=""):
        rospy.loginfo("move_shiver")
        self.publish(target="moving_platform", cmd=CMD.MOVE.SHIVER)
        self.exec_in_progress = False

    def move_avoid_front(self, modifier=""):
        rospy.loginfo("move_avoid_front")
        self.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
        sleep(.2)
        self.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
        sleep(.2)
        self.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
        sleep(.2)
        self.exec_in_progress = False

    def move_avoid_left(self, modifier=""):
        rospy.loginfo("move_avoid_left")
        self.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
        sleep(.2)
        self.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
        sleep(.2)
        self.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
        sleep(.2)
        self.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
        sleep(.2)
        self.publish(target="moving_platform", cmd=CMD.MOVE.RIGHT, arg=60)
        sleep(1)
        self.exec_in_progress = False

    def move_avoid_right(self, modifier=""):
        rospy.loginfo("move_avoid_right")
        self.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
        sleep(.2)
        self.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
        sleep(.2)
        self.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
        sleep(.2)
        self.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
        sleep(.2)
        self.publish(target="moving_platform", cmd=CMD.MOVE.LEFT, arg=60)
        sleep(1)
        self.exec_in_progress = False

    def move(self, modifier=()):
        rospy.loginfo("move %s" % str(modifier))
        # speed
        if "slower" in modifier:
            self.publish(target="moving_platform", cmd=CMD.MOVE.SPEED2)
        if "slow" in modifier:
            self.publish(target="moving_platform", cmd=CMD.MOVE.SPEED1)

        for direction in modifier:
            if direction == "forward":
                self.publish(target="moving_platform", cmd=CMD.MOVE.FORWARD)
            if direction == "backward":
                self.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
            if direction == "left":
                self.publish(target="moving_platform", cmd=CMD.MOVE.LEFT)
            if direction == "right":
                self.publish(target="moving_platform", cmd=CMD.MOVE.RIGHT)
            if direction == "stop":
                self.publish(target="moving_platform", cmd=CMD.MOVE.STOP)

        # restore speed
        if ("slower" in modifier) or ("slow" in modifier):
            self.publish(target="moving_platform", cmd=CMD.MOVE.SPEED3)

        self.exec_in_progress = False


class ConceptsPanic(Concept2CommandsInterpreterAbstract):
    def basic_panic(self, modifier=""):
        rospy.loginfo("basic_panic")
        self.publish(target="face_platform", cmd=CMD.FACE.SAD)

        panic_light = int(self.data.get("light", 0))
        current_light = panic_light
        rospy.loginfo("panic_light : 0x%x" % panic_light)
        self.publish(target="moving_platform", cmd=CMD.MOVE.SHIVER)
        sleep(2)
        while not (current_light > 1.2 * panic_light):
            self.publish(target="moving_platform", cmd=CMD.MOVE.FORWARD)
            sleep(.05)
            current_light = int(self.data.get("light", 0xFFFFFFFF))
            rospy.loginfo("current_light : 0x%x" % current_light)
        rospy.loginfo("out of loop")
        sleep(.5)
        self.publish(target="moving_platform", cmd=CMD.MOVE.STOP)
        sleep(.5)
        self.publish(target="moving_platform", cmd=CMD.MOVE.SHIVER)
        self.publish(target="face_platform", cmd=CMD.FACE.CALM)
        self.exec_in_progress = False


class ConceptsBacicReactions(Concept2CommandsInterpreterAbstract):
    def hello(self, modifier=""):
        rospy.loginfo("hello")
        self.publish(target="face_platform", cmd=CMD.FACE.CALM)
        self.exec_in_progress = False

    def bye(self, modifier=""):
        rospy.loginfo("bye")
        self.publish(target="face_platform", cmd=CMD.FACE.BLINK)
        self.exec_in_progress = False
