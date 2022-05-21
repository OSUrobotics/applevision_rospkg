#!/usr/bin/env python

import sys

import rospy
import moveit_commander
import moveit_msgs.msg
import tf
from std_msgs.msg import String
from sensor_msgs.msg import Range
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber

from applevision_rospkg.msg import PointWithCovarianceStamped, RegionOfInterestWithCovarianceStamped

class SynchronizerMinTick(ApproximateTimeSynchronizer):
    """
    Calls a callback if one of two conditions occur:
      1. A group of messages have timestamps within self.slop
      2. An above group has not occured within self.min_tick
    In other words, this class seeks to synchronize incoming messages with the constraint that
    we would also like the callback to be invoked at a minimum frequency. This behavior allows
    for graceful failure when a sensor stops publishing unexpectedly.

    If the min_tick timer elapses and not all messages are ready, the callback will be invoked
    with the earliest message from the highest priority subscriber (in order passed to the constructor),
    all topics that do not have a message recent enough will be filled with None.
    """

    def __init__(self, fs, queue_size, slop, min_tick):
        super(SynchronizerMinTick, self).__init__(fs, queue_size, slop)
        self.min_tick = rospy.Duration.from_sec(min_tick)
        self.last_tick = rospy.Time()
        self.tick_timer = rospy.Timer(self.min_tick, self._timerCallback, reset=True, oneshot=True)
    
    def signalMessage(self, *msg):
        if self.tick_timer:
            self.tick_timer.shutdown()

        self.last_tick = rospy.Time()
        self.tick_timer = rospy.Timer(self.min_tick, self._timerCallback, reset=True, oneshot=True)
        return super(SynchronizerMinTick, self).signalMessage(*msg)

    def _timerCallback(self, _):
        # signalMessage with whatever the latest data is
        with self.lock:
            found_msg = None
            for q in self.queues:
                if not q:
                    continue
                highest_stamp = max(q)
                if rospy.Time.now() - highest_stamp <= self.min_tick:
                    found_msg = q[highest_stamp]
            
            if not found_msg:
                # nothing has pinged within the tick rate
                rospy.logwarn('SynchronizerMinTick: no packets within min tick rate')
                ret = [None for _ in range(len(self.queues))]
            else:
                target_stamp = found_msg.header.stamp
                # collect all things that have pinged within the slop
                ret = []
                for q in self.queues:
                    if not q:
                        ret.append(None)
                        continue
                    best_stamp_diff, best_stamp = min((abs(target_stamp - s), s) for s in q)
                    if best_stamp_diff <= self.slop:
                        ret.append(q[best_stamp])
                    else:
                        ret.append(None)
            
            # empty queue and return
            self.signalMessage(*ret)
            for q in self.queues:
                q.clear()


class ApproachPlanner():
    def __init__(self, acSub, adSub):
        # Moveit Setup
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"

        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        self.tf_listener_ = tf.TransformListener()


        # CV Details

        self.CAMERA_RES = np.array((640, 360))
        self.CENTER_PT  = self.CAMERA_RES // 2

        self.aCLast = None
        self.aDLast = None

        self.tolerances = {
            'pixel' : 75,   # center vec
            'dist'  : 0.2,  # distance sensor
            'dead'  : 0.4,  # dead recking zone
            'miss'  : 20,   # count dropped
            'slop'  : 0.017,# sec timestamp off
            'tick'  : 1     # sec no data without action
        }

        self.distMissCount  = 0
        self.distDisabled   = False

        self.scale  = np.array((0.01,0.01))

        self.A      = None
        self.B      = None

        # synchronizer

        # registers tryPlan as the message callback.
        self.sync = SynchronizerMinTick([acSub, adSub], 10, self.tolerances['slop'], self.tolerances['min_tick'])
        self.sync.registerCallback(self.tryPlan)

    def toCenterCoord(self, q):
        return np.array((q.x + (q.w // 2), q.y + (q.h // 2)))

    # def aCCallback(self, res):
    #     if self.aCLast == None:
    #         self.aCLast = res
    #         self.tryPlan()
    #     else:
    #         if self.aDLast == None and self.distDisabled is False: self.distMissCount += 1
    #         if self.distMissCount > self.tolerances['miss']: self.distDisabled = True

    # def aDCallback(self, res):
    #     if self.aDLast == None:
    #         self.aDLast = res
    #         if self.distDisabled == True:
    #             self.distDisabled == False
    #             self.distMissCount = 0
    #         self.tryPlan()
    #     pass

    def tryPlan(self, cam_msg, dist_msg):
        # TODO: do something with cam_msg and dist_msg. Note that cam_msg and dist_msg can be None.
        # TODO: refactor now that messages aren't in self.aCLast and self.aDLast

        # Pre-Exit on Missing Sub
        if self.aCLast == None and (self.aDLast == None or self.distDisabled == True):
            rospy.logwarn("stuck doing nothing")
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

            cP = self.toCenterCoord(aC)

            if aD.range > self.tolerances['dist'] and aD.range > self.tolerances['dead']:
                # Advance closer to apple, this always happens after a potential trajectory correction step
                # Check if Camera is Centered
                if np.linalg.norm(self.CENTER_PT - cP) > self.tolerances['pixel']:
                    rospy.logwarn("Centering")
                    self.centerCamera(aC, aD)
                else:
                    msg = "Approaching, t-" + str(aD.range - self.tolerances['dist'])
                    rospy.logwarn(msg)
                    rospy.logwarn(aD.range)
                    self.forwardStep(aD)
                    pass
            elif aD.range > self.tolerances['dist'] and aD.range < self.tolerances['dead']:
                # Stop Checking for camera, start the full aproach
                msg = "Approaching (dead), t-" + str(aD.range - self.tolerances['dist'])
                rospy.logwarn(msg)
                self.forwardStep(aD)
            else:
                # Terminate Process
                rospy.logwarn("Done!")
                self.wrapUp()
                pass

    def wrapUp(self):
        rospy.logwarn("Terminating Motion Sequence")
        rospy.signal_shutdown("End Position Reached") 

    # Assumed Camera is in view when executed
    def centerCamera(self, c, d):
        # Check for target in FOV
        if (c.w or c.h):
            # target found

            # Transform pixel coordinates and generate pixel space vector to center

            CURCAM_PT   = np.array((c.x + c.w // 2, c.y + c.h // 2))

            if self.A is None or self.B is None:
                # 1st Move

                # Point (B) -> (A.t)
                self.A = CURCAM_PT
                self.B = self.CENTER_PT

                # X Component, Hip
                # End Effector Position [world]
                joint_goal = self.move_group.get_current_joint_values()

                # Normalization Hack: https://stackoverflow.com/questions/21030391/how-to-normalize-a-numpy-array-to-a-unit-vector
                # v / (np.linalg.norm(v) + 1e-16)

                v = (self.B - self.A)
                v = v / np.linalg.norm(v)

                rospy.logwarn(v)

                joint_goal[0] += (v[0] * self.scale[0])
                joint_goal[1] -= (v[1] * self.scale[1])

                self.move_group.go(joint_goal, wait=True)
                self.move_group.stop()

            # else:
            #     # 2nd Move

            #     # Points from previous run
            #     A = self.A
            #     B = self.B

            #     C = CURCAM_PT

            #     %rospy.logwarn([((np.linalg.norm(C - A))/(np.linalg.norm((B - A)*self.scale))), 1/((C - A)/((B - A)*self.scale))])

                

                # Wrap Up
                self.A = None
                self.B = None


    def forwardStep(self, d):
        
        rospy.logwarn((d.range - self.tolerances['dist']))
        if d.range > self.tolerances['dist']:

            pose_goal = self.move_group.get_current_pose(self.move_group.get_end_effector_link())

            pose_goal.pose.position.y += 0.1

            plan = self.move_group.set_pose_target(pose_goal)

            self.move_group.go(wait=True)

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

        aCsub = Subscriber('applevision/apple_camera', RegionOfInterestWithCovarianceStamped)
        aDsub = Subscriber('applevision/apple_dist', Range)
        aP = ApproachPlanner(aCsub, aDsub)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass




if __name__ == '__main__':
    main()