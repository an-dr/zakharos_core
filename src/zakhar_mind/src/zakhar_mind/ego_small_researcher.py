from time import sleep
import zakhar_common as com
from zakhar_pycore import zakhar__log as log
import zakhar_i2c_devices as dev
from .ego_like import EgoLikeNode


class EgoSmallResearcher(EgoLikeNode):
    def __init__(self, name="EgoSmallResearcher", log_level=log.INFO):
        EgoLikeNode.__init__(self, name=name, log_level=log_level)

    def main(self):
        while (1):
            for i in range(6): # 300 ms of right
                self.to_will("move_forward")
                sleep(.05)
            self.to_will("move_stop")
            sleep(1)
            for i in range(6):  # 300 ms of right
                self.to_will("move_right")
                sleep(.05)
            self.to_will("move_stop")
            sleep(1)


ego_small_researcher = EgoSmallResearcher()
