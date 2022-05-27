#!/usr/bin/python3

from typing import Optional
import rospy
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Range
from message_filters import Subscriber
from applevision_rospkg.msg import PointWithCovarianceStamped, RegionOfInterestWithConfidenceStamped
from applevision_rospkg.srv import Tf2Transform
from applevision_kalman.filter import KalmanFilter, EnvProperties
from helpers import RobustServiceProxy, ServiceProxyFailed, HeaderCalc, CameraInfoHelper, SynchronizerMinTick


def second_order_var_approx(fprime, fdoubleprime, ftrippleprime, xvar):
    # https://en.wikipedia.org/wiki/Taylor_expansions_for_the_moments_of_functions_of_random_variables
    return fprime**2 * xvar + 1 / 2 * fdoubleprime**2 * xvar**2 + fprime * ftrippleprime * xvar**2


class FilterHandler:

    def __init__(self, topic: str, kal: KalmanFilter) -> None:
        self.tf_get = RobustServiceProxy('Tf2Transform',
                                         Tf2Transform,
                                         persistent=True)
        self.cam_info = CameraInfoHelper('palm_camera/camera_info')
        self.p_out = rospy.Publisher(topic,
                                     PointWithCovarianceStamped,
                                     queue_size=10)
        self.viz_out = rospy.Publisher('applevision/est_apple_viz',
                                       PoseWithCovarianceStamped,
                                       queue_size=10)
        self.tf2_out = rospy.Publisher('tf', TFMessage, queue_size=10)
        self.kal = kal
        self._header = HeaderCalc('palm')
        self._gen = np.random.default_rng()

    def callback(self, dist: Optional[Range],
                 cam: Optional[RegionOfInterestWithConfidenceStamped]):
        try:
            dist_to_home = self.tf_get('palm', 'applevision_start_pos',
                                       rospy.Time(), rospy.Duration())
        except ServiceProxyFailed as e:
            rospy.logwarn(f'tf_get service proxy failed with error {e}')
            return
        trans: TransformStamped = dist_to_home.transform
        control = np.transpose(
            np.array([
                trans.transform.translation.x, trans.transform.translation.y,
                trans.transform.translation.z
            ]))

        x_est, p_est, var = self.kal.step_filter(cam, dist, control)

        trans_out = TransformStamped()
        trans_out.header = self._header.get_header()
        trans_out.header.frame_id = 'palm'
        trans_out.child_frame_id = 'applevision_est'
        trans_out.transform.rotation.w = 1
        trans_out.transform.translation.x = x_est[0]
        trans_out.transform.translation.y = x_est[1]
        trans_out.transform.translation.z = x_est[2]
        self.tf2_out.publish([trans_out])

        out = PointWithCovarianceStamped()
        out.header = self._header.get_header()
        out.camera_stamp = cam.header.stamp if cam else rospy.Time()
        out.distance_stamp = dist.header.stamp if dist else rospy.Time()
        out.point = x_est.tolist()
        out.covariance = p_est.flatten().tolist()
        self.p_out.publish(out)

        out_viz = PoseWithCovarianceStamped()
        out_viz.header = self._header.get_header()
        out_viz.pose.pose.position = trans_out.transform.translation
        out_viz.pose.pose.orientation.w = 1
        out_viz.pose.covariance = np.pad(p_est,
                                         ((0, 3), (0, 3))).flatten().tolist()
        self.viz_out.publish(out_viz)


def main():
    rospy.init_node('applevision_filter')
    rospy.wait_for_service('Tf2Transform')

    rospy.loginfo('Waiting for camera info...')
    cam_info = CameraInfoHelper('palm_camera/camera_info')
    cam_info.wait_for_camera_info()
    rospy.loginfo('Got it! Starting...')

    # TODO: Tune these
    env = EnvProperties(delta_t_ms=33,
                        accel_std=0.005,
                        starting_position=(0, 0, 1),
                        starting_std=.4,
                        z_std=.005,
                        backdrop_dist=.5,
                        apple_r=.035,
                        dist_fov_rad=np.deg2rad(20),
                        camera_info=cam_info.get_last_camera_info())
    kal_filter = KalmanFilter(
        env,
        std_range=1.5,
        too_close_cam=0.3,
        too_far_dist=7,
        appl_proportion_low=0.75,
        appl_proportion_high=0.9)
    rospy.logdebug(
        f'Kalman filter using environment {env} and filter {kal_filter}.')

    # input('Press any key to start Applevision...')
    main_proc = FilterHandler('applevision/est_apple_pos', kal_filter)
    dist = Subscriber('applevision/apple_dist', Range)
    camera = Subscriber('applevision/apple_camera',
                        RegionOfInterestWithConfidenceStamped)
    sync = SynchronizerMinTick([dist, camera], queue_size=10, slop=0.017, min_tick=0.2)
    sync.registerCallback(main_proc.callback)

    rospy.spin()


if __name__ == '__main__':
    main()
