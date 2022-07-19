#!/usr/bin/env python2

#from typing import Optional (removed for python2)
from enum import Enum#, auto
from itertools import count
def auto(it=count()):
    return it.next()
from threading import Lock
import traceback
#import sys

import rospy
import actionlib
from actionlib import GoalStatus
from moveit_commander import MoveGroupCommander
from message_filters import Subscriber
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, PositionConstraint, BoundingVolume, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import Range
from tf.listener import TransformListener

from applevision_rospkg.srv import Tf2TransformPoseStamped, AppleVis, AppleVisResponse
from applevision_rospkg.msg import RegionOfInterestWithConfidenceStamped, PointWithCovarianceStamped
from helpers import RobustServiceProxy, ServiceProxyFailed, SynchronizerMinTick


GROUP_NAME = 'manipulator'
EE_FRAME = 'palm'
WORLD_FRAME = 'world'
PLANNING_TIME = 1.0
SYNC_SLOP = 0.2
SYNC_TICK = 0.5
MOVE_TOLERANCE = 0.01
#LOG_PREFIX = sys.argv[1]

class MotionPlanner():
    def __init__(self):
        # Moveit setup
        self.move_group_action = actionlib.SimpleActionClient(
            'move_group', MoveGroupAction)
        self.tf_trans = RobustServiceProxy(
            'Tf2TransformPoseStamped', Tf2TransformPoseStamped, persistent=True, retry_count=5, backoff=0.05)

    def stop(self):
        self.move_group_action.cancel_all_goals()

    def is_in_motion(self): #-> bool:
        status = self.move_group_action.get_state()
        return status in [GoalStatus.ACTIVE, GoalStatus.PREEMPTING, GoalStatus.RECALLING, GoalStatus.PENDING]

    def start_move_to_pose(self, coords, tolerance):
        self.move_group_action.cancel_all_goals()

        # Translated from https://github.com/mikeferguson/moveit_python/blob/ros1/src/moveit_python/move_group_interface.py
        pose = PoseStamped()
        pose.pose.position.x = coords[0]
        pose.pose.position.y = coords[1]
        pose.pose.position.z = coords[2]
        pose.pose.orientation.w = 1
        pose.header.frame_id = EE_FRAME

        try:
            transformed_response = self.tf_trans(pose, WORLD_FRAME, rospy.Duration())
        except ServiceProxyFailed as e:
            raise RuntimeError('TF2 service proxy failed') #from e
        transformed_pose = transformed_response.transformed

        g = MoveGroupGoal()
        g.request.start_state.is_diff = True

        c1 = Constraints()
        p1 = PositionConstraint()
        p1.header.frame_id = WORLD_FRAME
        p1.link_name = EE_FRAME
        s = SolidPrimitive(dimensions=[tolerance**2], type=SolidPrimitive.SPHERE)
        b = BoundingVolume()
        b.primitives.append(s)
        b.primitive_poses.append(transformed_pose.pose)
        p1.constraint_region = b
        p1.weight = 1
        c1.position_constraints.append(p1)

        # locked forward for now
        o1 = OrientationConstraint()
        o1.header.frame_id = WORLD_FRAME
        o1.orientation.x = -0.707
        o1.orientation.w = 0.707
        o1.link_name = EE_FRAME
        o1.absolute_x_axis_tolerance = tolerance
        o1.absolute_y_axis_tolerance = tolerance
        o1.absolute_z_axis_tolerance = tolerance
        o1.weight = 1
        c1.orientation_constraints.append(o1)

        g.request.goal_constraints.append(c1)
        g.request.group_name = GROUP_NAME
        g.request.num_planning_attempts = 1
        g.request.allowed_planning_time = PLANNING_TIME
        g.planning_options.planning_scene_diff.is_diff = True
        g.planning_options.planning_scene_diff.robot_state.is_diff = True
        g.planning_options.look_around = False
        g.planning_options.replan = False

        self.move_group_action.send_goal(g)


class AppleApproach():
    CENTER_THRESH_XY = 0.02
    CAMERA_DEAD_THRESH_Z = 0.3
    DIST_VAR_GOOD_THRESH = 0.02**2
    STEP_DIST_Z = 0.05
    STOP_DIST_Z = 0.10
    ESTOP_DIST_Z = 0.06
    PALM_DIST_OFF_Y = -0.017 # TODO: fix from URDF

    class State(Enum):
        IDLE = auto()
        CENTER_IN_MOTION = auto()
        APPROACH_IN_MOTION = auto()
        DONE = auto()

    def __init__(self, planner): #: MotionPlanner):
        self.planner = planner
        self.state = self.State.IDLE
        self.next_state = None #: Optional[AppleApproach.State] = None
        self.running_lock = Lock()

        self._state_cb_table = {
            AppleApproach.State.IDLE: self.idle_callback,
            AppleApproach.State.CENTER_IN_MOTION: self.center_in_motion_callback,
            AppleApproach.State.APPROACH_IN_MOTION: self.approach_in_motion_callback
        }

    def die(self, msg):
        self.planner.stop()
        rospy.logfatal('{}'.format(msg)) #{LOG_PREFIX}{msg}')
        rospy.signal_shutdown('Death')

    def tick_callback(self, kal, cam, dist): #: Optional[PointWithCovarianceStamped], cam: Optional[RegionOfInterestWithConfidenceStamped], dist: Optional[Range]):
        if self.running_lock.locked():
            return
        try:
            with self.running_lock:
                if kal:
                    # TODO: this is hacky
                    kal.point = (kal.point[0], kal.point[1] + self.PALM_DIST_OFF_Y, kal.point[2])

                if self.state == AppleApproach.State.DONE:
                    rospy.loginfo(' Approach complete! Terminating...'.format()) #{LOG_PREFIX} Approach complete! Terminating...')
                    rospy.sleep(5)
                    rospy.signal_shutdown('All done!')
                ret = self._state_cb_table[self.state](kal, cam, dist)
                if ret:
                    next_state, msg = ret
                    rospy.loginfo('{}-> {}: {}'.format(self.state, next_state, msg)) #{LOG_PREFIX} {self.state} -> {next_state}: {msg}')
                    self.state = next_state
        except Exception as e:
            self.die('Caught exception {}:\n{}'.format(e, traceback.format_exc()))

    def idle_callback(self, kal, cam, dist): #Optional[PointWithCovarianceStamped], cam: Optional[RegionOfInterestWithConfidenceStamped], dist: Optional[Range]):
        if not kal or not cam:
            return None

        # If the camera is still useful according to the kalman filter and we need centering
        if kal.point[2] >= AppleApproach.CAMERA_DEAD_THRESH_Z \
            and (abs(kal.point[0]) > AppleApproach.CENTER_THRESH_XY or abs(kal.point[1]) > AppleApproach.CENTER_THRESH_XY):
            # we need a bounding box to continue, otherwise the filter has only a guess
            if not cam.w:
                return None
            # center the robot
            self.planner.start_move_to_pose((kal.point[0], kal.point[1], 0), MOVE_TOLERANCE)
            return (AppleApproach.State.CENTER_IN_MOTION, 'centering: {}, {}'.format(kal.point[0], kal.point[1]))

        # if we're close enough, stop
        if kal.point[2] <= AppleApproach.STOP_DIST_Z:
            return (AppleApproach.State.DONE, 'apple approach complete at distance {}'.format(kal.point[2]))

        # otherwise approach the apple slowly
        if kal.covariance[8] > AppleApproach.DIST_VAR_GOOD_THRESH:
            self.planner.start_move_to_pose((0, 0, min(kal.point[2] - AppleApproach.STOP_DIST_Z, AppleApproach.STEP_DIST_Z)), MOVE_TOLERANCE)
            return (AppleApproach.State.APPROACH_IN_MOTION, 'apple is centered: {}, {}, approaching slowly: {}'.format(kal.point[0], kal.point[1], kal.covariance[8]))
        else:
            self.planner.start_move_to_pose((0, 0, kal.point[2] - AppleApproach.STOP_DIST_Z), MOVE_TOLERANCE)
            return (AppleApproach.State.APPROACH_IN_MOTION, 'apple is centered: {}, {}, approaching quickly: {}').format(kal.point[0], kal.point[1], kal.covariance[8])

    def center_in_motion_callback(self, *_):
        if self.planner.is_in_motion():
            return None

        # if an error occurred, panic so we can reset
        status = self.planner.move_group_action.get_state()
        if status != GoalStatus.SUCCEEDED:
            self.die('Failed to move in center with status {} error status {}'.format(status, self.planner.move_group_action.get_goal_status_text()))

        # else we are forced to continue approaching since the camera is too noisy to learn anything
        return (AppleApproach.State.IDLE, 'done centering'.format())

    def approach_in_motion_callback(self, kal, cam, dist): #Optional[PointWithCovarianceStamped], cam: Optional[RegionOfInterestWithConfidenceStamped], dist: Optional[Range]):
        # sanity check: if the distance sensor reads under a certain value emergency stop
        if dist and dist.range < AppleApproach.ESTOP_DIST_Z:
            self.die('Detected obstruction at {}'.format(dist.range))

        # if the filter reads under the threshold, we're done! Better to stop early
        if kal and kal.point[2] <= AppleApproach.STOP_DIST_Z:
            self.planner.stop()
            return (AppleApproach.State.DONE, 'apple approach complete at distance {}'.format(kal.point[2]))

        # if we don't have the kalman filter wait another tick
        if self.planner.is_in_motion() or not kal:
            return None

        # if an error occurred, panic so we can reset
        status = self.planner.move_group_action.get_state()
        if status != GoalStatus.SUCCEEDED:
            self.die('Failed to move in approach with status {} error status {}'.format(status, self.planner.move_group_action.get_goal_status_text()))

        return (AppleApproach.State.IDLE, 'done approaching'.format())


def main():
    rospy.wait_for_service('Tf2TransformPoseStamped')

    planner = MotionPlanner()
    approach = AppleApproach(planner)
    print("ok")

    camera = Subscriber('applevision/apple_camera', RegionOfInterestWithConfidenceStamped, queue_size=10)
    dist = Subscriber('applevision/apple_dist', Range, queue_size=10)
    kal = Subscriber('applevision/est_apple_pos', PointWithCovarianceStamped, queue_size=10)
    print("ok1")
    min_tick = SynchronizerMinTick(
        [kal, camera, dist], queue_size=20, slop=SYNC_SLOP, min_tick=SYNC_TICK)
    min_tick.registerCallback(approach.tick_callback)
    print("ok2")

if __name__ == '__main__':
    rospy.init_node('applevision_motion')
    s = rospy.Service('applevision_motion', AppleVis, main())
    rospy.spin()