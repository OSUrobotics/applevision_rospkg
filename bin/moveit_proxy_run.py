#!/usr/bin/python

# Moveit! requires python2, so this file exposes a service that returns
# the robots position.

from __future__ import print_function

from applevision_kalman.srv import MoveitPose
import sys
import rospy
import moveit_commander

MOVE_GROUP = 'manipulator'

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('MoveitServer')
    move_group = moveit_commander.MoveGroupCommander(MOVE_GROUP)

    def get_pose(req):
        return move_group.get_current_pose().pose

    s = rospy.Service('MoveitPose', MoveitPose, get_pose)

    rospy.spin()

if __name__ == '__main__':
    main()