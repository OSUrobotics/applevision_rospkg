#!/usr/bin/python

from __future__ import print_function

import os
import subprocess
import sys
import select
from threading import Lock, Thread

import tf2_ros
import moveit_commander
import rospy
from message_filters import Cache, Subscriber
from moveit_msgs.msg import MoveGroupActionFeedback
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
from applevision_rospkg.srv import Tf2Transform


class ProxyStdout(Thread):
    def __init__(self, stdout, prefix, **kwargs):
        super(ProxyStdout, self).__init__(**kwargs)
        self.daemon = True
        self.prefix = prefix
        self.stdout = stdout

    def run(self):
        while not self.stdout.closed:
            line = self.stdout.readline()
            if not line:
                break
            rospy.loginfo(self.prefix + line.strip())

class ExpoDemo():
    BOUNDS_X = (-0.8, -0.2)
    BOUNDS_Y = (-1.2, 0)
    BOUNDS_Z = (1, 2)
    EXPO_DEMO_PROGRAM = os.path.join(os.path.dirname(__file__), 'applevision_motion.py')
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
        proc = subprocess.Popen(
            [self.EXPO_DEMO_PROGRAM], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
        ProxyStdout(proc.stdout, 'EXPO: DEMO {}:'.format(self.demo_seq)).start()
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
            is_out_bounds = coords.x < ExpoDemo.BOUNDS_X[0] or coords.x > ExpoDemo.BOUNDS_X[1] \
                or coords.y < ExpoDemo.BOUNDS_Y[0] or coords.y > ExpoDemo.BOUNDS_Y[1] \
                or coords.z < ExpoDemo.BOUNDS_Z[0] or coords.z > ExpoDemo.BOUNDS_Z[1]
            
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
                    rospy.sleep(1)
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
