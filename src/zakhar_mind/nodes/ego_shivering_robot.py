#!/usr/bin/env python3
import rospy
from zakhar_mind import ego_shivering_robot


if __name__ == '__main__':
    ego_shivering_robot.ego.start()
    rospy.spin()