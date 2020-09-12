#!/usr/bin/env python3
import rospy
from zakhar_pycore import zakhar__log as log
from zakhar_mind.concept_translator import concept_translator


if __name__ == '__main__':
    concept_translator.start()
    rospy.spin()