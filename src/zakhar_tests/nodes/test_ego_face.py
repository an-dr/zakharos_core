#!/usr/bin/env python3
import rospy
from zakhar_mind.ego_like import EgoLikeNode
from time import sleep


class EgoFaceTest(EgoLikeNode):
    def __init__(self, name="EgoFaceTest"):
        EgoLikeNode.__init__(self, name=name)

    def main(self):
        while (1):
            self.to_will("hello")
            sleep(1)
            self.to_will("bye")
            sleep(1)


ego = EgoFaceTest()

if __name__ == '__main__':
    ego.start()
    rospy.spin()
