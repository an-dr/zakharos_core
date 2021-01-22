#!/usr/bin/env python3
import rospy
from zakhar_mind.ego_like import EgoLikeNode
from time import sleep


class EgoEmotionReaderTest(EgoLikeNode):
    def __init__(self, name="EgoEmotionReaderTest"):
        EgoLikeNode.__init__(self, name=name)

    def main(self):
        while (1):
            sleep(1)


ego = EgoEmotionReaderTest()

if __name__ == '__main__':
    ego.start()
    rospy.spin()
