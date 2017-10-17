#!/usr/bin/python
"""
Created on 27/04/16

@author: Sammy Pfeiffer, Jordi Pages
@email: {sam.pfeiffer, jordi.pages}@pal-robotics.com

"""

import rospy
import rosbag
from sensor_msgs.msg import JointState
import yaml
import subprocess


class JointStateRecorder(object):
    def __init__(self, joint_state_topic='/joint_states'):
        rospy.loginfo("Init LearnFromJointState()")
        self.joint_states_topic = joint_state_topic
        # Creating a subscriber to joint states
        self.start_recording = False
        self.record_only_one = False
        self.single_waypoint_recorded = False
        self.joint_states_subs = rospy.Subscriber(
            self.joint_states_topic, JointState, self.joint_states_cb)
        rospy.loginfo("Subscribed to: " + self.joint_states_subs.resolved_name)
        self.current_rosbag_name = "uninitialized_rosbag_name"
        self.joint_states_accumulator = []

    def joint_states_cb(self, data):
        """joint_states topic cb"""        
        if self.start_recording:
            rospy.logdebug("Received joint_states:\n " + str(data))
            self.joint_states_accumulator.append(data)
            if self.record_only_one:
                self.start_recording = False
                self.single_waypoint_recorded = True

    def start(self, bag_name, single_waypoint, joints=[]):
        """Start the learning writting in the accumulator of msgs"""
        if single_waypoint:
            rospy.loginfo("Starting to record a single waypoint")
        else:
            rospy.loginfo("Starting to record in continuous mode")
        self.current_rosbag_name = bag_name
        self.record_only_one = single_waypoint
        self.single_waypoint_recorded = False
        self.start_recording = True        
        if single_waypoint:   # wait until a single joint_states message is recorded
            while not self.single_waypoint_recorded:
                rospy.sleep(0.1)


    def stop(self):
        """Stop the learning writting the bag into disk"""
        self.start_recording = False
        # self.joint_states_subs.unregister()
        rospy.loginfo("Recording in bag!")
        self.current_rosbag = rosbag.Bag(
            self.current_rosbag_name + '.bag', 'w')
        for js_msg in self.joint_states_accumulator:
            rospy.logdebug("Recording waypoint")
            self.current_rosbag.write(
                self.joint_states_topic, js_msg, t=js_msg.header.stamp)
        self.current_rosbag.close()
        rospy.loginfo("Motion finished and closed bag.")

        self.joint_states_accumulator = []
        return self.current_rosbag_name + '.bag'


class MotionGenerator(object):

    def __init__(self, bagname, joints, individual_waypoints_recorded, seconds_for_each_waypoint, frequency_to_downsample,
                 ignore_last_num_seconds=0.0, js_topic='/joint_states'):
        """Load gesture from the bag name given and remove jerkiness by downsampling"""
        # get bag info
        self.info_bag = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bagname],
                                                   stdout=subprocess.PIPE).communicate()[0])


        # Fill up traj with real trajectory points
        traj = []
        # Also fill up downsampled traj
        downsampled_traj = []
        bag = rosbag.Bag(bagname)
        first_point = True
        num_msgs = 0
        num_downsampled_data_points = 0
        for topic, msg, t in bag.read_messages(topics=[js_topic]):
            rospy.logdebug("Processing new message from rosbag")
            num_msgs += 1
            # Process interesting joints here
            names, positions = self.getNamesAndMsgList(joints, msg)
            if individual_waypoints_recorded:
                # Take all waypoints
                num_downsampled_data_points += 1
                downsampled_traj.append(positions)
            else:
                # Downsample the number of points if recorded using continuous mode
                if num_msgs % frequency_to_downsample == 0:
                    num_downsampled_data_points += 1
                    downsampled_traj.append(positions)
            # Append interesting joints here
            traj.append(positions)
            if first_point:
                # Store first point
                self.gesture_x0 = positions
                first_point = False

        bag.close()
        # Store last point
        self.gesture_goal = positions

        print str(len(traj)) + " points in example traj"
        print "Downsampled traj has: " + str(len(downsampled_traj)) + " points"

        rospy.loginfo("Joints:" + str(joints))
        rospy.loginfo("Initial pose:" + str(self.gesture_x0))
        rospy.loginfo("Final pose: " + str(self.gesture_goal))        

        self.motion_joints = joints
        self.motion_traj = downsampled_traj
        if individual_waypoints_recorded:
            self.motion_duration = num_downsampled_data_points * seconds_for_each_waypoint
        else:
            self.motion_duration = self.info_bag['duration']
        rospy.loginfo("Motion duration: " + str(self.motion_duration))


        # TODO: do play motion of it directly

        # gesture_dict = self.saveGestureYAML(
        #     bagname + ".yaml", bagname, joints, self.gesture_x0, self.gesture_goal, self.resp_from_makeLFDRequest, time_)
        # return gesture_dict

    def getNamesAndMsgList(self, joints, joint_state_msg):
        """ Get the joints for the specified group and return this name list and a list of it's values in joint_states
        Note: the names and values are correlated in position """

        list_to_iterate = joints
        curr_j_s = joint_state_msg
        ids_list = []
        msg_list = []
        rospy.logdebug("Current message: " + str(curr_j_s))
        for joint in list_to_iterate:
            idx_in_message = curr_j_s.name.index(joint)
            ids_list.append(idx_in_message)
            msg_list.append(curr_j_s.position[idx_in_message])
        rospy.logdebug(
            "Current position of joints in message: " + str(ids_list))
        rospy.logdebug("Current msg:" + str(msg_list))

        return list_to_iterate, msg_list

    def generatePlayMotion(self, motion_name="last_learnt_motion", play_speed=1.0):
        # Add the final generated traj as a play motion movement to re-do it if
        # we want
        play_motion_dict = {}
        play_motion_dict['joints'] = self.motion_joints
        play_motion_dict['meta'] = {
            'description': motion_name,
            'name': motion_name,
            'usage': 'demo'}
        play_motion_dict['points'] = []
        total_positions = len(self.motion_traj)
        rospy.loginfo("Number of positions in motion: " + str(total_positions) + "\n")
        timestep = (self.motion_duration / play_speed) / total_positions
        for idx, positions in enumerate(self.motion_traj):
            play_motion_dict['points'].append({'positions': positions,
                                               'time_from_start': float(idx) * timestep})

        rospy.loginfo("Motion looks like:\n" + str(play_motion_dict))
        rospy.loginfo(
            "Setting " + motion_name + " as a new play motion movement")
        rospy.set_param(
            "/play_motion/motions/" + motion_name , play_motion_dict)

        return play_motion_dict


class Executor(object):
    def __init__(self):
        pass


if __name__ == '__main__':
    rospy.init_node('learnonething')
    jsr = JointStateRecorder()
    jsr.start("test_motion")
    rospy.sleep(10.0)
    bag_name = jsr.stop()

    rospy.loginfo("Generating...")
    joints = ['head_1_joint', 'head_2_joint']
    mg = MotionGenerator(bag_name, joints)

    mg.generatePlayMotion("LBD 0.5X", play_speed=0.5)
    mg.generatePlayMotion("LBD 1X", play_speed=1.0)
    mg.generatePlayMotion("LBD 2X", play_speed=2.0)
    mg.generatePlayMotion("LBD 3X", play_speed=3.0)
