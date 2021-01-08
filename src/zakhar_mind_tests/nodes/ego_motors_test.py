#!/usr/bin/env python3
import rospy
from zakhar_mind.ego_like import EgoLikeNode
from time import sleep

class EgoMotorsTest(EgoLikeNode):
    def __init__(self, name="EgoMotorsTest"):
        EgoLikeNode.__init__(self, name=name)

    def main(self):
        while (1):
            self.to_will("move_forward")
            sleep(1)
            self.to_will("move_shiver")
            sleep(2)
            self.to_will("move_stop")
            sleep(1)


if __name__ == '__main__':
    ego = EgoMotorsTest()
    ego.start()
    rospy.spin()