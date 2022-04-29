#!/usr/bin/env python
import math
import sys
import copy
from tokenize import group
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from random import random
import os
import subprocess, shlex, psutil

from math import pi

import std_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler

from std_msgs.msg import String
from sensor_msgs.msg import Range
import csv

import numpy as np

from applevision_rospkg.msg import PointWithCovarianceStamped, RegionOfInterestWithCovarianceStamped

import tf


class approachPlanner(object):
    def __init__(self):
        super(approachPlanner, self).__init__()

        # Moveit Setup
        
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"

        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        self.success_or_failure_publilsher = rospy.Publisher('/success_or_failure', std_msgs.msg.String, queue_size=20)

        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print "============ Available Planning Groups:", self.robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print self.robot.get_current_state()
        print ""

        # applevision camera

        self.tf_listener_ = tf.TransformListener()

        self.CAMERA_RES = (640, 360)
        self.CENTER_PT  = (self.CAMERA_RES[0]//2, self.CAMERA_RES[1]//2)

        self.aCLast = None
        self.aDLast = None

        self.pixelTolerance  =   0
        self.distTolerance   =   0.3

        self.pixelStep

    def aCCallback(self, res):
        if self.aCLast == None:
            self.aCLast = res
            self.tryPlan()
        pass

    def aDCallback(self, res):
        if self.aDLast == None:
            self.aDLast = res
            self.tryPlan()
        pass

    def tryPlan(self):
        # Pre-Exit on Missing Sub
        if self.aCLast == None or self.aDLast == None:
            return

        # Pull values for inspection

        aC = self.aCLast
        aD = self.aDLast

        # Sanitize workspace

        self.aCLast = None
        self.aDLast = None

        # Check for target in FOV
        if (aC.w or aC.h):
            # target found

            # Transform pixel coordinates and generate pixel space vector to center
            CURCAM_PT   = (aC.x + aC.w // 2, aC.y + aC.h // 2)
            CURCAM_VEC  = (self.CENTER_PT[0] - CURCAM_PT[0], self.CENTER_PT[1] - CURCAM_PT[1])

            # Check if Camera is Centered
            if np.linalg.norm(np.array(CURCAM_VEC)) > self.pixelTolerance:
                self.centerCamera(aC)

            # Advance closer to apple, this always happens after a potential trajectory correction step

            if aD.range > self.distTolerance:
                self.forwardStep()
                pass
            else:
                # Terminate Process
                # self.wrapUp()
                pass


            # rospy.logwarn(CURCAM_VEC)
            # rospy.logwarn(aD)

    def wrapUp(self):
        rospy.logwarn("Terminating Motion Sequence")
        rospy.signal_shutdown("End Position Reached") 

    # Assumed Camera is in view when executed
    def centerCamera(self, c):
        # Check for target in FOV
        if (c.w or c.h):
            # target found

            # Transform pixel coordinates and generate pixel space vector to center
            CURCAM_PT   = (c.x + c.w // 2, c.y + c.h // 2)
            CURCAM_VEC  = (self.CENTER_PT[0] - CURCAM_PT[0], self.CENTER_PT[1] - CURCAM_PT[1])

            mag = np.linalg.norm(np.array(CURCAM_VEC))

            CURCAM_VEC = (CURCAM_VEC[0]/mag,CURCAM_VEC[1]/mag)

            # Generated direction to move in, generating move it move

            # End Effector Position [world]
            eefP = self.move_group.get_current_pose(self.move_group.get_end_effector_link())

            # Convert to frame palm

            t = self.tf_listener_.getLatestCommonTime("/world", "/palm")
            p1 = geometry_msgs.msg.PoseStamped()
            p1.header.frame_id = "/palm"
            p1.pose = eefP.pose

            # End Effector Position [Palm]

            eefP_palm = self.tf_listener_.transformPose("/palm", p1)

            stepSize = 1

            # eefP_palm.pose.position.x += (CURCAM_VEC[0] * stepSize, CURCAM_VEC[1] * stepSize)
            # eefP_palm.pose.position.y += (CURCAM_VEC[0] * stepSize, CURCAM_VEC[1] * stepSize)

            # Convert [Palm] back to [world]

            t = self.tf_listener_.getLatestCommonTime("/palm", "/world")
            p2 = geometry_msgs.msg.PoseStamped()
            p2.header.frame_id = "/world"
            p2.pose = eefP_palm.pose

            # End Effector Position [world] (updated)

            eefP_world = self.tf_listener_.transformPose("/world", p2)

            rospy.logwarn_once([eefP, eefP_palm, eefP_world])

    def forwardStep(self):

        

        pass

def main():
    
    try:
        rospy.init_node('applevision_motion')
        rospy.wait_for_service('Tf2Transform')

        aP = approachPlanner()

        aCpub = rospy.Subscriber('applevision/apple_camera', RegionOfInterestWithCovarianceStamped, aP.aCCallback, queue_size=10)
        aDPub = rospy.Subscriber('applevision/apple_dist', Range, aP.aDCallback, queue_size=10)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass




if __name__ == '__main__':
    main()