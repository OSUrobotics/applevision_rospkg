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

        self.tf_listener_ = tf.TransformListener()

        self.CAMERA_RES = (640, 360)
        self.CENTER_PT  = (self.CAMERA_RES[0]//2, self.CAMERA_RES[1]//2)

        self.aCLast = None
        self.aDLast = None

        self.pixelTolerance  =   10
        self.distTolerance   =   0.3

        # self.pixelStep

    def toCenterCoord(q):
        return (q.x + q.w // 2, q.y + q.h // 2)

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
            if np.abs(CURCAM_VEC[0]) > self.pixelTolerance or np.abs(CURCAM_VEC[1]) > self.pixelTolerance:
                rospy.logwarn("Centering")
                self.centerCamera(aC, CURCAM_VEC)
            else:   # Advance closer to apple, this always happens after a potential trajectory correction step
                if aD.range > self.distTolerance:
                    msg = "Approaching, t-" + str(aD.range - self.distTolerance)
                    rospy.logwarn(msg)
                    self.forwardStep(aD)
                    pass
                else:
                    # Terminate Process
                    rospy.logwarn("Done!")
                    # self.wrapUp()
                    pass

    def wrapUp(self):
        rospy.logwarn("Terminating Motion Sequence")
        rospy.signal_shutdown("End Position Reached") 

    # Assumed Camera is in view when executed
    def centerCamera(self, c, CURCAM_VEC):
        # Check for target in FOV
        if (c.w or c.h):
            # target found

            CURCAM_VECn = (CURCAM_VEC[0]/np.linalg.norm(CURCAM_VEC),CURCAM_VEC[1]/np.linalg.norm(CURCAM_VEC))

            if np.abs(CURCAM_VEC[0]) > self.pixelTolerance:

                # End Effector Position [world]
                eefP = self.move_group.get_current_pose(self.move_group.get_end_effector_link())

                # We can get the joint values from the group and adjust some of the values:
                joint_goal = self.move_group.get_current_joint_values()
                joint_goal[0] += (CURCAM_VECn[0] * 0.1)

                self.move_group.go(joint_goal, wait=True)
                self.move_group.stop()

            if np.abs(CURCAM_VEC[1]) > self.pixelTolerance:

                # End Effector Position [world]
                eefP = self.move_group.get_current_pose(self.move_group.get_end_effector_link())

                # We can get the joint values from the group and adjust some of the values:
                joint_goal = self.move_group.get_current_joint_values()
                joint_goal[1] += (CURCAM_VECn[0] * 0.01)

                self.move_group.go(joint_goal, wait=True)
                self.move_group.stop()

    

    def forwardStep(self, d):
        
        rospy.logwarn((d.range - self.distTolerance))
        if d.range > self.distTolerance:

            pose_goal = self.move_group.get_current_pose(self.move_group.get_end_effector_link())

            pose_goal.pose.position.y += 0.1

            plan = self.move_group.set_pose_target(pose_goal)

            rospy.logwarn("Premotion...")

            self.move_group.go(wait=True)

            rospy.logwarn("Postmotion...")

            self.move_group.stop()


        # Convert to frame palm

        # t = self.tf_listener_.getLatestCommonTime("/world", "/palm")
        # p1 = geometry_msgs.msg.PoseStamped()
        # p1.header.frame_id = "/palm"
        # p1.pose = eefP.pose

        # End Effector Position [Palm]

        # eefP_palm = self.tf_listener_.transformPose("/palm", p1)

        # stepSize = 1

        # eefP_palm.pose.position.x += (CURCAM_VEC[0] * stepSize, CURCAM_VEC[1] * stepSize)
        # eefP_palm.pose.position.y += (CURCAM_VEC[0] * stepSize, CURCAM_VEC[1] * stepSize)

        # Convert [Palm] back to [world]

        # t = self.tf_listener_.getLatestCommonTime("/palm", "/world")
        # p2 = geometry_msgs.msg.PoseStamped()
        # p2.header.frame_id = "/world"
        # p2.pose = eefP_palm.pose

        # End Effector Position [world] (updated)

        # eefP_world = self.tf_listener_.transformPose("/world", p2)

        # rospy.logwarn_once([eefP, eefP_palm, eefP_world])

        pass

def main():
    
    try:
        rospy.init_node('applevision_motion')
        rospy.wait_for_service('Tf2Transform')

        aP = approachPlanner()

        aCpub = rospy.Subscriber('applevision/apple_camera', RegionOfInterestWithCovarianceStamped, aP.aCCallback, queue_size=1)
        aDPub = rospy.Subscriber('applevision/apple_dist', Range, aP.aDCallback, queue_size=10)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass




if __name__ == '__main__':
    main()