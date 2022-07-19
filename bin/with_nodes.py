#!/usr/bin/env python2

import logging
Logger = logging.getLogger(name="name")
handler = logging.StreamHandler()
Logger.addHandler(handler)
Logger.setLevel(logging.DEBUG)

import rospy

from applevision_rospkg.srv import AppleVis, MoveGroup

rospy.init_node('with_nodes')

# initial position
joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
initial = [0, -.58, -2.26, -.44, 1.62, .74]
#apple coords
apple = [-.51, -.16, 1.3]

# move to initial position
rospy.wait_for_service('move_group')
beginning = rospy.ServiceProxy('move_group', MoveGroup)
result = beginning(joints, initial)

# apple approach
rospy.wait_for_service('applevision_motion')
main = rospy.ServiceProxy('applevision_motion', AppleVis)
response = main(True)

# get final position

# check if it's nearby