#!/usr/bin/env python3
import rospy
from zakhar_mind.ego_small_researcher import ego_small_researcher


if __name__ == '__main__':
    ego_small_researcher.start()
    rospy.spin()