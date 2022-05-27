#!/usr/bin/python3

import rospy
import math
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, Point
from applevision_rospkg.msg import PointWithCovarianceStamped
from applevision_rospkg.srv import Tf2Transform
from helpers import RobustServiceProxy, ServiceProxyFailed, HeaderCalc


# TODO: rework to use camerainfo
class CamVizHandler:
    # camera focal length ~ 11mm?
    # camera sensor size 5449umx3072um
    # resolution 640x360
    # camera only supports 672x380, so image is cropped slightly
    CAMERA_RES = (640, 360)
    CAMERA_SENSOR = (5449*(640/672)*1e-6, 3072*(360/380)*1e-6)
    CAMERA_FOCAL = 11e-3

    def __init__(self) -> None:
        self.tf_get = RobustServiceProxy('Tf2Transform', Tf2Transform, persistent=True)
        self.p_out = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self._header = HeaderCalc('palm')
        self._gen = np.random.default_rng()

    def make_cam_fov_marker(self, z_extrap: float, cam_msg):
        # draw the complete camera FOV
        # https://www.edmundoptics.com/knowledge-center/application-notes/imaging/understanding-focal-length-and-field-of-view/
        CAM_FOV_ACROSS = 2*math.atan(self.CAMERA_SENSOR[0]/(2*self.CAMERA_FOCAL))
        CAM_FOV_UPDOWN = 2*math.atan(self.CAMERA_SENSOR[1]/(2*self.CAMERA_FOCAL))
        X_AT_Z_EXTRAP = z_extrap*math.tan(CAM_FOV_ACROSS/2)
        Y_AT_Z_EXTRAP = z_extrap*math.tan(CAM_FOV_UPDOWN/2)

        cam_mark = Marker()
        cam_mark.header.frame_id = 'palm'
        cam_mark.header.stamp = rospy.Time.now()
        cam_mark.ns = 'applevision_cam_fov'
        cam_mark.id = 0
        cam_mark.type = Marker.LINE_LIST
        cam_mark.action = Marker.ADD
        cam_mark.scale.x = 0.01
        cam_mark.pose.orientation.w = 1
        cam_mark.color.r = 1
        cam_mark.color.a = 0.5
        cam_mark.points = [
            # draw cone
            Point(0, 0, 0), Point(X_AT_Z_EXTRAP, Y_AT_Z_EXTRAP, z_extrap),
            Point(0, 0, 0), Point(-X_AT_Z_EXTRAP, Y_AT_Z_EXTRAP, z_extrap),
            Point(0, 0, 0), Point(-X_AT_Z_EXTRAP, -Y_AT_Z_EXTRAP, z_extrap),
            Point(0, 0, 0), Point(X_AT_Z_EXTRAP, -Y_AT_Z_EXTRAP, z_extrap),
            # draw box at end of cone
            Point(X_AT_Z_EXTRAP, Y_AT_Z_EXTRAP, z_extrap), Point(-X_AT_Z_EXTRAP, Y_AT_Z_EXTRAP, z_extrap),
            Point(-X_AT_Z_EXTRAP, Y_AT_Z_EXTRAP, z_extrap), Point(-X_AT_Z_EXTRAP, -Y_AT_Z_EXTRAP, z_extrap),
            Point(-X_AT_Z_EXTRAP, -Y_AT_Z_EXTRAP, z_extrap), Point(X_AT_Z_EXTRAP, -Y_AT_Z_EXTRAP, z_extrap),
            Point(X_AT_Z_EXTRAP, -Y_AT_Z_EXTRAP, z_extrap), Point(X_AT_Z_EXTRAP, Y_AT_Z_EXTRAP, z_extrap),
        ]

        # draw the bounding box
        box_mark = Marker()
        box_mark.header.frame_id = 'palm_camera'
        box_mark.header.stamp = rospy.Time.now()
        box_mark.ns = 'applevision_cam_box'
        box_mark.id = 0
        box_mark.type = Marker.LINE_LIST
        box_mark.action = Marker.ADD
        box_mark.scale.x = 0.005
        box_mark.pose.orientation.w = 1
        box_mark.pose.position.z = z_extrap
        box_mark.color.g = 1
        box_mark.color.a = 0.5

        top_left = np.array((cam_msg.x - self.CAMERA_RES[0]/2, cam_msg.y - self.CAMERA_RES[1]/2))
        bottom_right = top_left + (cam_msg.w, cam_msg.h)
        top_left_m = top_left/self.CAMERA_RES*(X_AT_Z_EXTRAP, Y_AT_Z_EXTRAP)*2
        bottom_right_m = bottom_right/self.CAMERA_RES*(X_AT_Z_EXTRAP, Y_AT_Z_EXTRAP)*2

        box_mark.points = [
            Point(top_left_m[0], top_left_m[1], 0), Point(bottom_right_m[0], top_left_m[1], 0),
            Point(bottom_right_m[0], top_left_m[1], 0), Point(bottom_right_m[0], bottom_right_m[1], 0),
            Point(bottom_right_m[0], bottom_right_m[1], 0), Point(top_left_m[0], bottom_right_m[1], 0),
            Point(top_left_m[0], bottom_right_m[1], 0), Point(top_left_m[0], top_left_m[1], 0)
        ]

        return [box_mark, cam_mark]

    def callback(self, cam):
        # draw the cameras FOV and bounding box
        dist_to_apple = self.tf_get('applevision_target', 'palm_camera', rospy.Time(), rospy.Duration())
        print(dist_to_apple)
        markers = self.make_cam_fov_marker(dist_to_apple.transform.transform.translation.z, cam)
        for marker in markers:
            self.p_out.publish(marker)


class KalmanVizHandler:
    def __init__(self) -> None:
        self.tf_get = RobustServiceProxy('Tf2Transform', Tf2Transform, persistent=True)
        self.p_out = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    def callback(self, res: PointWithCovarianceStamped):
        try:
            dist_to_apple = self.tf_get('applevision_est', 'palm', rospy.Time(), rospy.Duration())
        except ServiceProxyFailed as e:
            rospy.logwarn(f'tf_get service proxy failed with error {e}')
            return
        trans: TransformStamped = dist_to_apple.transform
        # draw an arrow from the grabber to where we think the apple is
        arrow_mark = Marker()
        arrow_mark.header.frame_id = 'palm'
        arrow_mark.header.stamp = rospy.Time.now()
        arrow_mark.ns = 'applevision_arrow'
        arrow_mark.id = 0
        arrow_mark.type = Marker.LINE_LIST
        arrow_mark.action = Marker.ADD
        arrow_mark.scale.x = 0.005
        arrow_mark.pose.orientation.w = 1
        arrow_mark.color.g = 1
        arrow_mark.color.a = 0.5
        arrow_mark.points = [
            Point(0, 0, 0), trans.transform.translation
        ]
        self.p_out.publish(arrow_mark)

def main():
    rospy.init_node('applevision_vizualizer')
    rospy.wait_for_service('Tf2Transform')

    # main_proc = CamVizHandler()
    # camera = rospy.Subscriber('applevision/apple_camera', RegionOfInterestWithCovarianceStamped, main_proc.callback, queue_size=10)
    kal_proc = KalmanVizHandler()
    kal = rospy.Subscriber('applevision/est_apple_pos', PointWithCovarianceStamped, kal_proc.callback, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    main()
