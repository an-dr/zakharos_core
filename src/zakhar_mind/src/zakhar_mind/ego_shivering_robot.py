#!/usr/bin/python
import rospy
from zakhar_mind.ego_like import EgoLikeNode
from time import sleep

class EgoShiveringRobot(EgoLikeNode):
    def __init__(self, name="EgoShiveringRobot"):
        EgoLikeNode.__init__(self, name=name)

    def main(self):
        while (1):
            self.to_will("move_shiver")
            sleep(5)

ego = EgoShiveringRobot()

if __name__ == '__main__':
    ego.start()
    rospy.spin()