#!/usr/bin/python

from __future__ import print_function

import os
import subprocess
import sys
from threading import Lock
import math

import tf2_ros
import moveit_commander
import rospy
from message_filters import Cache, Subscriber
from moveit_msgs.msg import MoveGroupActionFeedback
from visualization_msgs.msg import Marker


def quaternion_multiply(quaternion1, quaternion0):
    x0, y0, z0, w0 = quaternion0
    x1, y1, z1, w1 = quaternion1
    return ([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0])

class ExpoDemo():
    BOUNDS_X = (-0.8, -0.2)
    BOUNDS_Y = (-1.2, 0)
    BOUNDS_Z = (1, 2)
    BOUND_ORIENTATION = 10
    ORIENTATION_TARGET = (-0.707, 0, 0, 0.707)
    EXPO_DEMO_PROGRAM = os.path.join(os.path.dirname(__file__), 'applevision_motion.py')
    EXPO_BAG = '/home/avl/new_ws/bags/approach-ok.bag'
    CHECK_DURATION = rospy.Duration.from_sec(0.1)
    GROUP_NAME = 'manipulator'
    HOME_POSITION = 'up'
    CLOSE_ENOUGH = 0.05
    TICK_TIMEOUT = 10

    @classmethod
    def make_allowed_box_mark(cls):
        box_mark = Marker()
        box_mark.header.frame_id = 'world'
        box_mark.header.stamp = rospy.Time.now()
        box_mark.ns = 'applevision_expo_box'
        box_mark.id = 0
        box_mark.type = Marker.CUBE
        box_mark.action = Marker.ADD
        box_mark.pose.position.x = (cls.BOUNDS_X[1] - cls.BOUNDS_X[0])/2 + cls.BOUNDS_X[0]
        box_mark.pose.position.y = (cls.BOUNDS_Y[1] - cls.BOUNDS_Y[0])/2 + cls.BOUNDS_Y[0]
        box_mark.pose.position.z = (cls.BOUNDS_Z[1] - cls.BOUNDS_Z[0])/2 + cls.BOUNDS_Z[0]
        box_mark.scale.x = cls.BOUNDS_X[1] - cls.BOUNDS_X[0]
        box_mark.scale.y = cls.BOUNDS_Y[1] - cls.BOUNDS_Y[0]
        box_mark.scale.z = cls.BOUNDS_Z[1] - cls.BOUNDS_Z[0]
        box_mark.pose.orientation.w = 1
        box_mark.color.g = 1
        box_mark.color.a = 0.2
        return box_mark

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.move_group = moveit_commander.MoveGroupCommander(self.GROUP_NAME)
        home_pose_dict = self.move_group.get_named_target_values(self.HOME_POSITION)
        self.home_pose = [home_pose_dict[key] for key in self.move_group.get_active_joints()]
        self.moveit_feedback_sub = Subscriber('/move_group/feedback', MoveGroupActionFeedback)
        self.moveit_feedback = Cache(self.moveit_feedback_sub, cache_size=5)
        self.marker_out = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.running_lock = Lock()

        self.demo_process = None
        self.idle_count = 0
        self.demo_seq = 0
        self.is_homing = True

        self.last_state = {}

        rospy.on_shutdown(self.kill_robot)

    def make_control_system_process(self):
        # proc = subprocess.Popen([self.EXPO_DEMO_PROGRAM, 'EXPO: DEMO {}:'.format(self.demo_seq)])
        proc = subprocess.Popen(['/opt/ros/melodic/bin/rosbag', 'play', ExpoDemo.EXPO_BAG])
        self.demo_seq += 1
        return proc

    def start(self):
        # TODO call this
        rospy.loginfo('EXPO: Starting Expo Demo! Hopefully it works...')
        self.check_timer = rospy.Timer(self.CHECK_DURATION, self.check_cb)

    def die_gracefully(self, msg):
        self.kill_robot()
        rospy.logfatal(msg)
        rospy.sleep(30)
        sys.exit(1)

    def kill_robot(self):
        if self.demo_process:
            try:
                self.demo_process.kill()
            except Exception as e:
                rospy.logwarn('DEMO: Faled to kill with warning: {}'.format(e))
            self.demo_process = None
        self.move_group.stop()

    def go_home(self):
        if self.demo_is_running():
            rospy.logwarn('EXPO: killing demo to go home')
            self.kill_robot()
        self.move_group.set_named_target(self.HOME_POSITION)
        self.move_group.go(wait=False)
        self.is_homing = True

    def is_moving(self):
        if len(self.moveit_feedback.cache_msgs) == 0:
            # rospy.logwarn('EXPO: ambiguous moving check')
            return False
        return self.moveit_feedback.cache_msgs[-1].feedback.state != "IDLE"

    def is_home(self):
        if self.is_moving():
            return False
        cur_pose = self.move_group.get_current_joint_values()
        for cur, target in zip(cur_pose, self.home_pose):
            if abs(cur - target) > self.CLOSE_ENOUGH:
                return False
        return True

    def demo_is_running(self):
        return self.demo_process is not None and self.demo_process.poll() is None

    def is_position_out(self, coords):
        return coords.x < ExpoDemo.BOUNDS_X[0] or coords.x > ExpoDemo.BOUNDS_X[1] \
                or coords.y < ExpoDemo.BOUNDS_Y[0] or coords.y > ExpoDemo.BOUNDS_Y[1] \
                or coords.z < ExpoDemo.BOUNDS_Z[0] or coords.z > ExpoDemo.BOUNDS_Z[1]
    
    def is_orientation_out(self, quat):
        # find the smallest angle between this quaternion and the target, and check it's within range
        new = quaternion_multiply((-quat.x, -quat.y, -quat.z, quat.w), self.ORIENTATION_TARGET)
        angle = abs(180.0 - math.degrees(2*math.atan2(math.sqrt(new[0]**2 + new[1]**2 + new[2]**2), new[3])))
        return angle > self.BOUND_ORIENTATION

    def check_cb(self, *args):
        if self.running_lock.locked():
            return

        with self.running_lock:
            # compute bounds check
            try:
                where_in_world = self.tf_buffer.lookup_transform('world', 'palm', rospy.Time())
            except Exception as e:
                self.die_gracefully('EXPO: tf failed with error {}. Probably time to restart the system.'.format(e))
            coords = where_in_world.transform.translation
            is_out_bounds = self.is_position_out(coords)
            orientation = where_in_world.transform.rotation
            is_out_orientation = self.is_orientation_out(orientation)

            # count idle ticks
            if self.demo_is_running() and not self.is_moving():
                self.idle_count += 1
            else:
                self.idle_count = 0

            # reset homing
            if not self.is_moving():
                self.is_homing = False

            # log state for debugging
            state = {'demo': self.demo_is_running(), 'moving': self.is_moving(), 'home': self.is_home(), 'homing': self.is_homing}
            if state != self.last_state:
                rospy.logdebug('EXPO: state {}'.format(state))
                self.last_state = state

            if not self.is_homing and self.demo_process:
                # perform sanity checks if we're not homing and we've started at least once
                if is_out_bounds:
                    # the robot is not going home and is outside the safe zone
                    coord_str = str(coords).replace('\n', ' ')
                    rospy.logwarn('EXPO: bounds check failed with coordinates {}, terminating and reseting...'.format(coord_str))
                    self.kill_robot()
                elif is_out_orientation:
                    # the robot is doing disco moves
                    rospy.logwarn('EXPO: bounds check failed for orientation, terminating and reseting...')
                    self.kill_robot()
                elif self.idle_count > self.TICK_TIMEOUT:
                    # the robot has been idle (not controlled by this program) for too long
                    rospy.logwarn('EXPO: demo got stuck, restarting...')
                    self.kill_robot()
                    self.idle_count = 0
            
            if not self.is_moving() and not self.demo_is_running():
                # if we're stopped move the state forward
                if self.is_home():
                    # the robot is home and ready to start the demo
                    rospy.loginfo('EXPO: Starting demo program...')
                    self.demo_process = self.make_control_system_process()
                    rospy.sleep(0.1)
                    if not self.demo_is_running():
                        self.die_gracefully('EXPO: demo crashed on launch, system cannot recover.')
                else:
                    # the demo is crashed and the robot is stuck in another position
                    rospy.loginfo('EXPO: Sending robot home...')
                    self.go_home() 

        # publish a marker for debugging
        self.marker_out.publish(self.make_allowed_box_mark())


def main():
    rospy.init_node('applevision_expo_demo')
    rospy.wait_for_service('Tf2Transform')

    # TODO: watch distance and camera to make sure they're kosher

    expo_demo = ExpoDemo()
    rospy.sleep(1)
    timer = rospy.Timer(rospy.Duration.from_sec(1), expo_demo.check_cb)

    rospy.spin()


if __name__ == '__main__':
    main()
