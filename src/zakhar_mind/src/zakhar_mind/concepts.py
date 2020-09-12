import rospy
from time import sleep
import zakhar_common as com
from zakhar_pycore import zakhar__log as log
import zakhar_i2c_devices as dev
from zakhar_i2c_devices.moving_platform import CMD_BACKWARD, CMD_FORWARD, CMD_LEFT, CMD_RIGHT, CMD_SHIVER, CMD_STOP
from zakhar_i2c_devices.face_platform import CMD_ANGRY, CMD_BLINK, CMD_CALM, CMD_HAPPY, CMD_SAD
from zakhar_msgs import msg, srv
from .concept_translator_abstract import ConceptTranslatorAbstract

class ConceptsMove(ConceptTranslatorAbstract):
    def move_forward(self):
        self.l.info("move_stop")
        self.publish(target="moving_platform", arg_str="w", arg_a=0, arg_b=CMD_FORWARD)
        self.exec_in_progress = False

    def move_backward(self):
        self.l.info("move_stop")
        self.publish(target="moving_platform", arg_str="w", arg_a=0, arg_b=CMD_BACKWARD)
        self.exec_in_progress = False

    def move_left(self):
        self.l.info("move_stop")
        self.publish(target="moving_platform", arg_str="w", arg_a=0, arg_b=CMD_LEFT)
        self.exec_in_progress = False

    def move_right(self):
        self.l.info("move_stop")
        self.publish(target="moving_platform", arg_str="w", arg_a=0, arg_b=CMD_RIGHT)
        self.exec_in_progress = False

    def move_stop(self):
        self.l.info("move_stop")
        self.publish(target="moving_platform", arg_str="w", arg_a=0, arg_b=CMD_STOP)
        self.exec_in_progress = False

    def move_shiver(self):
        self.l.info("move_shiver")
        self.publish(target="moving_platform", arg_str="w", arg_a=0, arg_b=CMD_SHIVER)
        self.exec_in_progress = False


class ConceptsPanic(ConceptTranslatorAbstract):
    def basic_panic(self):
        self.l.info("basic_panic")
        self.publish(target="face_platform", arg_str="w", arg_a=0, arg_b=CMD_SAD)

        if False:
            self.publish(target="moving_platform", arg_str="w", arg_a=0, arg_b=CMD_FORWARD)
            sleep(.2)
            self.publish(target="moving_platform", arg_str="w", arg_a=0, arg_b=CMD_BACKWARD)
            sleep(.2)
            self.publish(target="moving_platform", arg_str="w", arg_a=0, arg_b=CMD_FORWARD)
            sleep(.2)
            self.publish(target="moving_platform", arg_str="w", arg_a=0, arg_b=CMD_BACKWARD)
            sleep(.2)
            self.publish(target="moving_platform", arg_str="w", arg_a=0, arg_b=CMD_FORWARD)
            sleep(.2)
            self.publish(target="moving_platform", arg_str="w", arg_a=0, arg_b=CMD_BACKWARD)
            sleep(.2)
            self.publish(target="moving_platform", arg_str="w", arg_a=0, arg_b=CMD_FORWARD)
            sleep(.2)
            self.publish(target="moving_platform", arg_str="w", arg_a=0, arg_b=CMD_BACKWARD)
            sleep(.2)
            self.publish(target="moving_platform", arg_str="w", arg_a=0, arg_b=CMD_STOP)
        else:
            panic_light = self.data.get("light", 0)
            current_light = panic_light
            print("panic_light : 0x%x" % panic_light)
            self.publish(target="moving_platform", arg_str="w", arg_a=0, arg_b=CMD_SHIVER)
            sleep(3)
            # self.publish(target="moving_platform", arg_str="w", arg_a=0, arg_b=CMD_FORWARD)
            while not (current_light > 1.2 * panic_light):
                self.publish(target="moving_platform", arg_str="w", arg_a=0, arg_b=CMD_FORWARD)
                sleep(.05)
                current_light = self.data.get("light", 0xFFFFFFFF)
                print("current_light : 0x%x" % current_light)
            print("out of cirle")
            sleep(.5)
            self.publish(target="moving_platform", arg_str="w", arg_a=0, arg_b=CMD_STOP)
            sleep(.5)
            self.publish(target="moving_platform", arg_str="w", arg_a=0, arg_b=CMD_SHIVER)
        self.publish(target="face_platform", arg_str="w", arg_a=0, arg_b=CMD_CALM)
        self.exec_in_progress = False


class ConceptsBacicReactions(ConceptTranslatorAbstract):
    def hello(self):
        self.l.info("hello")
        self.publish(target="face_platform", arg_str="w", arg_a=0, arg_b=CMD_CALM)
        self.exec_in_progress = False

    def bye(self):
        self.l.info("bye")
        self.publish(target="face_platform", arg_str="w", arg_a=0, arg_b=CMD_BLINK)
        self.exec_in_progress = False
