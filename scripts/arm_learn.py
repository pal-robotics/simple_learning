#!/usr/bin/python
"""
Created on 27/04/16

@author: Sammy Pfeiffer
@email: sam.pfeiffer@pal-robotics.com

"""

import rospy
from simple_learn import JointStateRecorder, MotionGenerator

if __name__ == '__main__':
    rospy.init_node('learnarm')
    jsr = JointStateRecorder()
    jsr.start("arm_test_motion")
    rospy.sleep(10.0)
    bag_name = jsr.stop()

    rospy.loginfo("Generating...")
    joints = []
    for i in range(1, 8):
        joints.append("arm_" + str(i) + "_joint")

    mg = MotionGenerator(bag_name, joints)

    mg.generatePlayMotion("LBD_HALFX", play_speed=0.5)
    mg.generatePlayMotion("LBD_1X", play_speed=1.0)
    mg.generatePlayMotion("LBD_2X", play_speed=2.0)
    mg.generatePlayMotion("LBD_3X", play_speed=3.0)
