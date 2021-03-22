import rospy
from .command_concept_abstract import CommandConceptAbstract
from zakhar_pycore import dev as zdev
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

    def _pre_main_analysis(self) -> bool:
        """ Adds modifiers if necessary nased on emotions, finds contradictions """
        return True

    def _main(self):
        """ Interaction with devices """
        rospy.loginfo("move %s" % str(self._mod))
        # speed
        if self.is_modifier(self.modifiers.slower):
            self.cmd(target=zdev.moving_platform.name, cmd=zdev.moving_platform.cmd.SPEED2)
        if self.is_modifier(self.modifiers.slow):
            self.cmd(target=zdev.moving_platform.name, cmd=zdev.moving_platform.cmd.SPEED1)

        for direction in self._mod:
            if direction == self.modifiers.forward:
                self.cmd(target=zdev.moving_platform.name, cmd=zdev.moving_platform.cmd.FORWARD)
            if direction == self.modifiers.backward:
                self.cmd(target=zdev.moving_platform.name, cmd=zdev.moving_platform.cmd.BACKWARD)
            if direction == self.modifiers.left:
                self.cmd(target=zdev.moving_platform.name, cmd=zdev.moving_platform.cmd.LEFT)
            if direction == self.modifiers.right:
                self.cmd(target=zdev.moving_platform.name, cmd=zdev.moving_platform.cmd.RIGHT)
            if direction == self.modifiers.stop:
                self.cmd(target=zdev.moving_platform.name, cmd=zdev.moving_platform.cmd.STOP)

        # restore speed
        if self.is_modifier(self.modifiers.slow) or self.is_modifier(self.modifiers.slower):
            self.cmd(target=zdev.moving_platform.name, cmd=zdev.moving_platform.cmd.SPEED3)


class Shiver(CommandConceptAbstract):
    name = "shiver"

    def _pre_main_analysis(self) -> bool:
        """ Adds modifiers if necessary nased on emotions, finds contradictions """
        return True

    def _main(self):
        """ Interaction with devices """
        self.cmd(target=zdev.moving_platform.name, cmd=zdev.moving_platform.cmd.SHIVER)


class Avoid(CommandConceptAbstract):
    name = "avoid"

    class modifiers:
        front = "front"
        left = "left"
        right = "right"

    def _pre_main_analysis(self) -> bool:
        """ Adds modifiers if necessary nased on emotions, finds contradictions """
        if (self.is_modifier(self.modifiers.left)
                or self.is_modifier(self.modifiers.right)) and not self.is_modifier(self.modifiers.front):
            self._mod.append(self.modifiers.front)

        if self.is_modifier(self.modifiers.left) and self.is_modifier(self.modifiers.right):
            self._mod.remove(self.modifiers.left)
            self._mod.remove(self.modifiers.right)

        return True

    def _main(self):
        """ Interaction with devices """
        if self.is_modifier(self.modifiers.front):
            self.cmd(target=zdev.moving_platform.name, cmd=zdev.moving_platform.cmd.BACKWARD)
            self.sleep(.2)
            self.cmd(target=zdev.moving_platform.name, cmd=zdev.moving_platform.cmd.BACKWARD)
            self.sleep(.2)
            self.cmd(target=zdev.moving_platform.name, cmd=zdev.moving_platform.cmd.BACKWARD)
            self.sleep(.2)
        if self.is_modifier(self.modifiers.left):
            self.cmd(target=zdev.moving_platform.name, cmd=zdev.moving_platform.cmd.RIGHT, arg=60)
            self.sleep(1)
        if self.is_modifier(self.modifiers.right):
            self.cmd(target=zdev.moving_platform.name, cmd=zdev.moving_platform.cmd.LEFT, arg=60)
            self.sleep(1)


class BasicPanic(CommandConceptAbstract):
    name = "basic_panic"

    def _pre_main_analysis(self) -> bool:
        """ Adds modifiers if necessary nased on emotions, finds contradictions """
        return True

    def _main(self):
        """ Interaction with devices """
        rospy.loginfo("basic_panic")
        self.cmd(target=zdev.face_module.name, cmd=zdev.face_module.cmd.SAD)

        panic_light = int(self._c2c.data.get("light", 0))
        current_light = panic_light
        rospy.loginfo("panic_light : 0x%x" % panic_light)
        self.cmd(target=zdev.moving_platform.name, cmd=zdev.moving_platform.cmd.SHIVER)
        self.sleep(2)
        while not (current_light > 1.2 * panic_light):
            self.cmd(target=zdev.moving_platform.name, cmd=zdev.moving_platform.cmd.FORWARD)
            self.sleep(.05)
            current_light = int(self._c2c.data.get("light", 0xFFFFFFFF))
            rospy.loginfo("current_light : 0x%x" % current_light)
        rospy.loginfo("out of loop")
        self.sleep(.5)
        self.cmd(target=zdev.moving_platform.name, cmd=zdev.moving_platform.cmd.STOP)
        self.sleep(.5)
        self.cmd(target=zdev.moving_platform.name, cmd=zdev.moving_platform.cmd.SHIVER)
        self.cmd(target=zdev.face_module.name, cmd=zdev.face_module.cmd.CALM)


class Express(CommandConceptAbstract):
    name = "express"

    class modifiers:
        hello = "hello"
        bye = "bye"

    def _pre_main_analysis(self) -> bool:
        """ Adds modifiers if necessary nased on emotions, finds contradictions """
        return True

    def _main(self):
        """ Interaction with devices """
        if self.is_modifier(self.modifiers.hello):
            self.cmd(target=zdev.face_module.name, cmd=zdev.face_module.cmd.CALM)
        elif self.is_modifier(self.modifiers.bye):
            self.cmd(target=zdev.face_module.name, cmd=zdev.face_module.cmd.BLINK)
