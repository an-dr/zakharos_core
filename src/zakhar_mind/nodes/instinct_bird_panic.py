#!/usr/bin/env python3
import rospy
from zakhar_mind.instinct_bird_panic import instinct


if __name__ == '__main__':
    instinct.start()
    rospy.spin()