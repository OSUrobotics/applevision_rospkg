#!/usr/bin/python3

import math
import itertools
import time
import rospy
import cv2
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import Range
from geometry_msgs.msg import Pose, TransformStamped, Point, Quaternion, Vector3
from tf2_msgs.msg import TFMessage
from applevision_rospkg.msg import RegionOfInterestWithCovarianceStamped
from applevision_rospkg.srv import Tf2Transform
from applevision_kalman.model import ConeSensorModel
from helpers.robust_serviceproxy import RobustServiceProxy, ServiceProxyFailed

DISTANCE_PERIOD = rospy.Duration.from_sec(1/32)
CAMERA_PERIOD = rospy.Duration.from_sec(1/32)
APPLE_R = 0.04


def make_tf(ts, frame_id, child_frame_id, x, y, z, rx=0, ry=0, rz=0, rw=1) -> TransformStamped:
    tf = TransformStamped()
    tf.header.frame_id = frame_id
    tf.header.stamp = ts
    tf.child_frame_id = child_frame_id
    tf.transform.translation.x = x
    tf.transform.translation.y = y
    tf.transform.translation.z = z
    tf.transform.rotation.x = rx
    tf.transform.rotation.y = ry
    tf.transform.rotation.z = rz
    tf.transform.rotation.w = rw
    return tf


class HeaderCalc:
    def __init__(self, frame_id: str):
        self.frame_id = frame_id
        self._seq = 0

    def get_header(self):
        now = rospy.get_rostime()
        fake_header = Header(seq=self._seq, stamp=now, frame_id=self.frame_id)
        if self._seq >= 4294967295:  # uint32 max
            self._seq = 0
        else:
            self._seq += 1

        return fake_header


class DistPub:
    def __init__(self, frame_id: str, topic: str) -> None:
        self.pub = rospy.Publisher(topic, Range, queue_size=10)
        self.tf_get = RobustServiceProxy('Tf2Transform', Tf2Transform, persistent=True)
        self._header_calc = HeaderCalc(frame_id)
        self._cone_sensor_model = ConeSensorModel(1.5, APPLE_R, 0.001, np.random.default_rng())

    def callback(self, *args):
        # compute a fake distance based off of the robots position
        # TODO: drop random data points
        try:
            dist_to_apple = self.tf_get('fake_apple', 'fake_grabber', rospy.Time(), rospy.Duration())
        except ServiceProxyFailed as e:
            rospy.logwarn(f'tf_get service proxy failed with error {e}')
            return
        trans: TransformStamped = dist_to_apple.transform
        vect = (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
        dist = self._cone_sensor_model.measure(vect)
        self.pub.publish(header=self._header_calc.get_header(), radiation_type=Range.INFRARED, field_of_view=ConeSensorModel.FOV_RAD, min_range=0, max_range=1300, range=dist)


class CamPub:
    # camera focal length ~ 11mm?
    # camera sensor size 5449umx3072um
    # resolution 640x360
    # camera only supports 672x380, so image is cropped slightly
    CAMERA_RES = (640, 360)
    CAMERA_SENSOR = (5449*(640/672)*1e-6, 3072*(360/380)*1e-6)
    CAMERA_FOCAL = 11e-3

    def __init__(self, frame_id: str, topic: str) -> None:
        self.pub = rospy.Publisher(topic, RegionOfInterestWithCovarianceStamped, queue_size=10)
        self.tf_get = RobustServiceProxy('Tf2Transform', Tf2Transform, persistent=True)
        self._header_calc = HeaderCalc(frame_id)
        self._rng = np.random.default_rng()

        f_x_pix = self.CAMERA_FOCAL/self.CAMERA_SENSOR[0]*640
        f_y_pix = self.CAMERA_FOCAL/self.CAMERA_SENSOR[1]*360
        self._camera_matrix = np.array([
            [f_x_pix, 0, 0],
            [0, f_y_pix, 0],
            [0, 0, 1]
        ])
        sphere_points = []
        for x, y in itertools.product((-APPLE_R, APPLE_R), repeat=2):
            sphere_points.append([x, y, -APPLE_R])
        self._sphere_points = np.array(sphere_points)

    def callback(self, *args):
        # TODO: flange frame is looking at robot
        # compute a fake bounding box based off of the robots position
        try:
            dist_to_apple = self.tf_get('fake_apple', 'fake_grabber_cam', rospy.Time(), rospy.Duration())
        except ServiceProxyFailed as e:
            rospy.logwarn(f'tf_get service proxy failed with error {e}')
            return
        trans: TransformStamped = dist_to_apple.transform
        relative_to_apple = np.array((trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z))
        points_relative_to_apple = self._sphere_points + relative_to_apple
        # compute the ideal bounding box we'd see given the radius of the camera
        # assume no rotation of arm
        points, _ = cv2.projectPoints(points_relative_to_apple, np.eye(3), np.array([0.0, 0.0, 0.0]), self._camera_matrix, None)
        # offset each point to bring the origin to the corner of the image
        points += np.array([self.CAMERA_RES[0]/2, self.CAMERA_RES[1]/2])
        # add noise to each point in pixels
        points += self._rng.normal(0, 10, points.shape)
        # estimate a bounding box by finding the max and min of x, y
        x_hold = (0, 1e9)
        y_hold = (0, 1e9)
        for (x, y), in points:
            x = np.clip(x, 0, 640)
            y = np.clip(y, 0, 360)
            x_hold = (max(x, x_hold[0]), min(x, x_hold[1]))
            y_hold = (max(y, y_hold[0]), min(y, y_hold[1]))

        # compute bounding box and publish
        x_var = 100 if x_hold[0] != x_hold[1] else np.inf
        y_var = 100 if y_hold[0] != y_hold[1] else np.inf
        self.pub.publish(
            header=self._header_calc.get_header(),
            x=round(x_hold[1]), y=round(y_hold[1]),
            w=round(x_hold[0]-x_hold[1]), h=round(y_hold[0]-y_hold[1]),
            x_var=x_var, y_var=y_var, w_var=2*x_var, h_var=2*y_var)


class FakeAppleFramePub:
    def __init__(self, apple_tf, pub):
        self.apple_tf = apple_tf
        self.pub = pub

    def callback(self, *args):
        self.apple_tf.header.stamp = rospy.Time.now()
        self.pub.publish(TFMessage([self.apple_tf]))


def make_fake_apple() -> CollisionObject:
    solid = SolidPrimitive(SolidPrimitive.SPHERE, [APPLE_R])
    pose = Pose()
    pose.orientation.w = 1

    fake_apple = CollisionObject()
    fake_apple.header.frame_id = 'fake_apple'
    fake_apple.id = 'fake_apple'
    fake_apple.primitives = [solid]
    fake_apple.primitive_poses = [pose]
    fake_apple.operation = CollisionObject.ADD

    return fake_apple


def main():
    rospy.init_node('applevision_fake_sensor_data')
    rospy.wait_for_service('apply_planning_scene')
    rospy.wait_for_service('Tf2Transform')

    rospy.loginfo('Adding fake apple to rviz...')
    planning_service = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)
    fake_apple = make_fake_apple()
    planning_scene = PlanningScene()
    planning_scene.is_diff = True
    planning_scene.world.collision_objects = [fake_apple]
    planning_service(planning_scene)

    rospy.loginfo('Starting fake sensor data...')
    rospy.wait_for_service('Tf2Transform')
    dist_pub = DistPub('fake_grabber_dist', 'applevision/apple_dist')
    cam_pub = CamPub('fake_grabber_cam', 'applevision/apple_camera')
    dist_timer = rospy.Timer(DISTANCE_PERIOD, dist_pub.callback)
    cam_timer = rospy.Timer(CAMERA_PERIOD, cam_pub.callback)

    while not rospy.is_shutdown():
        time.sleep(0.1)

    dist_timer.shutdown()
    cam_timer.shutdown()


if __name__ == '__main__':
    main()
