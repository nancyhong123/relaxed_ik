#! /usr/bin/env python
'''
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 7/1/18

PLEASE DO NOT CHANGE CODE IN THIS FILE.  IF TRYING TO SET UP RELAXEDIK, PLEASE REFER TO start_here.py INSTEAD
AND FOLLOW THE STEP-BY-STEP INSTRUCTIONS THERE.  Thanks!
'''
######################################################################################################

import rospy
from RelaxedIK.relaxedIK import RelaxedIK
from relaxed_ik.msg import EEPoseGoals, JointAngles
from std_msgs.msg import Float32
from RelaxedIK.Utils.colors import bcolors
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, PoseStamped
import rospkg
import os
import numpy as np

if __name__ == '__main__':
    rospy.init_node('kinematics_eval_node')
    # while eepg == None: continue
    trial  = 0
    control_method = 'orientation'
    # control_method = 'optimization'
    # control_method = 'centering'

    r = rospkg.RosPack()
    path = r.get_path('testing')
    results_location = path + '/test_results/orientation_3d/kinematic/'
    # self.error_filepath = results_location + tracking_method + "_trial_" + str(trial) + ".csv"
    # path_type = "ideal"
    # path_type = "noisy_25"
    # path_type = "noisy_36"
    path_type = "noisy_71"
    if not os.path.exists(results_location):
        os.makedirs(results_location)
    error_filepath = results_location + 'kinematic_results_' + control_method+'_' + path_type +'_trial_'+ str(trial) + ".csv"
    results_fp = open(error_filepath,'ab')
    results_fp.truncate(0)


    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        # print("waiting for robot state msg")

        # print("waiting for robot joint msg")
        # joint_state = rospy.wait_for_message('/dvrk/PSM2/state_joint_current', JointState, timeout=None)

        veloctity_msg = rospy.wait_for_message('/dvrk/PSM2/twist_body_current', TwistStamped, timeout=None)

        position_msg = rospy.wait_for_message('/dvrk/PSM2/position_cartesian_current',PoseStamped, timeout=None)

        
        # print("robot state:", robot_state)
        # print("joint state:", joint_state)
        # print("twist: ",twist )
        # x = joint_state.position[0:7]
        # print("joint state:",x)
        msg_time = veloctity_msg.header.stamp
        time = msg_time.to_sec()
        position = position_msg.pose.position
        velocity = veloctity_msg.twist.linear
        np.savetxt(results_fp ,np.array([[time, position.x,position.y, position.z, velocity.x ,velocity.y,velocity.z]]), delimiter=',', fmt='%s', newline='\n')
        rate.sleep()

    results_fp.close()





