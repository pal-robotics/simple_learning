#!/usr/bin/python
"""
Created on 06/05/16

@author: Sammy Pfeiffer, Jordi Pages
@email: {sam.pfeiffer, jordi.pages}@pal-robotics.com

"""

import rospy
from simple_learn import JointStateRecorder, MotionGenerator
#from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from simple_learning.srv import Learn, LearnRequest, LearnResponse
import yaml


class LearnServer(object):
    def __init__(self):
        self.jsr = JointStateRecorder()
        self.learning = False
        self.bag_name = "/tmp/" + "lbd_motion"
        self.last_joints_to_learn = None
        self.recording_individual_waypoints = False
        self.seconds_for_each_waypoint = 0 # only used when recording individual waypoints
        self.start_srv = rospy.Service('/learn_by_demo_start',
                                       Learn,
                                       self.start_cb)
        self.stop_srv = rospy.Service('/learn_by_demo_stop',
                                      Learn,
                                      self.stop_cb)                                      
        rospy.loginfo("Learning by demo service ready.")

    def start_cb(self, req):
        self.recording_individual_waypoints = req.record_single_waypoint
        self.seconds_for_each_waypoint = req.seconds_for_each_waypoint
        self.jsr.start(self.bag_name, req.record_single_waypoint)
        self.last_joints_to_learn = req.joints_to_learn
        rospy.loginfo("Will learn from joints: " + str(req.joints_to_learn))
        self.learning = True
        return LearnResponse()

    def stop_cb(self, req):
        if self.learning:
            self.jsr.stop()
            self.learning = False
        if self.last_joints_to_learn:
            joints = self.last_joints_to_learn
            self.last_joints_to_learn = None
        else:
            rospy.loginfo("On start didnt say from which joints to learn... getting from stop")
            if len(req.joints_to_learn) < 1:
                rospy.logerr("Didnt give joint names to learn in start or stop call, aborting")
                return LearnResponse()
            else:
                joints = req.joints_to_learn
        print "Stop cb, joints to learn:" + str(joints)
        frequency_to_downsample = 10
        mg = MotionGenerator(self.bag_name + ".bag", joints,
                             individual_waypoints_recorded = self.recording_individual_waypoints,
                             seconds_for_each_waypoint = self.seconds_for_each_waypoint,
                             frequency_to_downsample = 10)
        mg.generatePlayMotion("LBD_HALFX", play_speed=0.5)
        motion_dict = mg.generatePlayMotion("LBD_1X", play_speed=1.0)
        mg.generatePlayMotion("LBD_2X", play_speed=2.0)
        resp = LearnResponse()
        resp.motion = yaml.dump(motion_dict)
        return resp


if __name__ == '__main__':
    rospy.init_node('learning_by_demo_server')
    ls = LearnServer()
    rospy.spin()
