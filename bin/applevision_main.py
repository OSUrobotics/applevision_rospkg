#!/usr/bin/python3

import rospy
import math
import numpy as np
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Range
from message_filters import ApproximateTimeSynchronizer, Subscriber
from applevision_rospkg.msg import PointWithCovarianceStamped, RegionOfInterestWithCovarianceStamped
from applevision_rospkg.srv import Tf2Transform
from applevision_kalman.filter import KalmanFilter, EnvProperties


class HeaderCalc:
    def __init__(self, frame_id: str):
        self.frame_id = frame_id
        self._seq = 0

    def get_header(self):
        now = rospy.Time.now()
        fake_header = Header(seq=self._seq, stamp=now, frame_id=self.frame_id)
        if self._seq >= 4294967295:  # uint32 max
            self._seq = 0
        else:
            self._seq += 1

        return fake_header


class MainHandler:
    CAMERA_RES = (640, 360)
    CAMERA_SENSOR = (5449*(640/672)*1e-6, 3072*(360/380)*1e-6)
    CAMERA_FOCAL = 11e-3

    def __init__(self, tf_get: rospy.ServiceProxy, p_out: rospy.Publisher, tf2_out: rospy.Publisher, kal: KalmanFilter) -> None:
        self.tf_get = tf_get
        self.p_out = p_out
        self.tf2_out = tf2_out
        self.kal = kal
        self._header = HeaderCalc('fake_grabber')
        self._gen = np.random.default_rng()

    def callback(self, dist: Range, cam: RegionOfInterestWithCovarianceStamped):
        dist_to_apple = self.tf_get('fake_grabber', 'start_pos', rospy.Time(), rospy.Duration())
        trans: TransformStamped = dist_to_apple.transform
        control = np.transpose(np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]))

        # compute apple x, y based off of the bounding box width/height avg
        # TODO: how to fix this? it's unreliable
        # TODO: improve varience calculations
        apple_avg_dim = (cam.w + cam.h)/2
        est_frame_width_m = self.CAMERA_RES[0]/apple_avg_dim*self.kal.env.apple_r
        est_frame_height_m = est_frame_width_m*(self.CAMERA_RES[1]/self.CAMERA_RES[0])
        est_x = cam.x/self.CAMERA_RES[0]*est_frame_width_m
        est_y = cam.y/self.CAMERA_RES[1]*est_frame_height_m

        # publish the kalman filters predicted apple distance
        # TODO: fix zero bounding box error
        x_est, p_est, var = self.kal.step_filter(
            (est_x, est_y, dist.range),
            cam.x_var*(est_frame_width_m/self.CAMERA_RES[0]),
            cam.y_var*(est_frame_height_m/self.CAMERA_RES[1]),
            control)

        trans_out = TransformStamped()
        trans_out.header = self._header.get_header()
        trans_out.child_frame_id = 'apple'
        trans_out.transform.rotation.w = 1
        trans_out.transform.translation.x = x_est[0]
        trans_out.transform.translation.y = x_est[1]
        trans_out.transform.translation.z = x_est[2]
        self.tf2_out.publish([trans_out])

        out = PointWithCovarianceStamped()
        out.header = self._header.get_header()
        out.camera_stamp = cam.header.stamp
        out.distance_stamp = dist.header.stamp
        out.point = x_est.tolist()
        out.covariance = p_est.flatten().tolist()
        self.p_out.publish(out)


def main():
    rospy.init_node('applevision_main')
    rospy.wait_for_service('Tf2Transform')

    # TODO: Tune these
    env = EnvProperties(delta_t_ms=33,
                        accel_std=1,
                        starting_position=(0, 0, 1),
                        starting_std=.4,
                        z_std=.005,
                        backdrop_dist=.5,
                        apple_r=.080,
                        dist_fov_rad=np.deg2rad(25))
    kal_filter = KalmanFilter(env, 1.5, 0.75, 0.9)
    rospy.logdebug(f'Kalman filter using environment {env} and filter {kal_filter}.')

    # input('Press any key to start Applevision...')
    # TODO: service proxy is unreliable, needs a retry mechanism
    get_pose = rospy.ServiceProxy('Tf2Transform', Tf2Transform)
    dist = Subscriber('applevision/apple_dist', Range)
    camera = Subscriber('applevision/apple_camera', RegionOfInterestWithCovarianceStamped)
    p = rospy.Publisher('applevision/est_apple_pos', PointWithCovarianceStamped, queue_size=10)
    tf2_p = rospy.Publisher('tf', TFMessage, queue_size=10)

    main_proc = MainHandler(get_pose, p, tf2_p, kal_filter)

    # allow half a frame of distance between two stamped data points
    sync = ApproximateTimeSynchronizer([dist, camera], 10, 0.017)
    sync.registerCallback(main_proc.callback)

    rospy.spin()


if __name__ == '__main__':
    main()
