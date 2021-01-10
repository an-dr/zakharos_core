import rospy
from time import sleep
from .concept2commands_interpreter_abstract import Concept2CommandsInterpreterAbstract
from zakhar_pycore.constants import CMD


class ConceptsMove(Concept2CommandsInterpreterAbstract):
    def move_forward(self, modificator=""):
        rospy.loginfo("move_forward")
        self.publish(target="moving_platform", cmd=CMD.MOVE.FORWARD)
        self.exec_in_progress = False

    def move_backward(self, modificator=""):
        rospy.loginfo("move_stop")
        self.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
        self.exec_in_progress = False

    def move_left(self, modificator=""):
        rospy.loginfo("move_left")
        self.publish(target="moving_platform", cmd=CMD.MOVE.LEFT)
        self.exec_in_progress = False

    def move_right(self, modificator=""):
        rospy.loginfo("move_right")
        self.publish(target="moving_platform", cmd=CMD.MOVE.RIGHT)
        self.exec_in_progress = False

    def move_stop(self, modificator=""):
        rospy.loginfo("move_stop")
        self.publish(target="moving_platform", cmd=CMD.MOVE.STOP)
        self.exec_in_progress = False

    def move_shiver(self, modificator=""):
        rospy.loginfo("move_shiver")
        self.publish(target="moving_platform", cmd=CMD.MOVE.SHIVER)
        self.exec_in_progress = False

    def move_avoid_front(self, modificator=""):
        rospy.loginfo("move_avoid_front")
        self.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
        sleep(.2)
        self.exec_in_progress = False

    def move_avoid_left(self, modificator=""):
        rospy.loginfo("move_avoid_left")
        self.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
        sleep(.2)
        self.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
        sleep(.2)
        self.publish(target="moving_platform", cmd=CMD.MOVE.RIGHT, arg=60)
        sleep(1)
        self.exec_in_progress = False

    def move_avoid_right(self, modificator=""):
        rospy.loginfo("move_avoid_right")
        self.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
        sleep(.2)
        self.publish(target="moving_platform", cmd=CMD.MOVE.BACKWARD)
        sleep(.2)
        self.publish(target="moving_platform", cmd=CMD.MOVE.RIGHT, arg=60)
        sleep(1)
        self.exec_in_progress = False


class ConceptsPanic(Concept2CommandsInterpreterAbstract):
    def basic_panic(self, modificator=""):
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
    def hello(self, modificator=""):
        rospy.loginfo("hello")
        self.publish(target="face_platform", cmd=CMD.FACE.CALM)
        self.exec_in_progress = False

    def bye(self, modificator=""):
        rospy.loginfo("bye")
        self.publish(target="face_platform", cmd=CMD.FACE.BLINK)
        self.exec_in_progress = False
