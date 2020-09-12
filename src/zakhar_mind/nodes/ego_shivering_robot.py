#!/usr/bin/env python3
import rospy
from zakhar_mind import ego_shivering_robot
from zakhar_pycore import zakhar__log as log


if __name__ == '__main__':
    ego_shivering_robot.ego.start()
    rospy.spin()