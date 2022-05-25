#!/usr/bin/python3

import rospy
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Range
from message_filters import ApproximateTimeSynchronizer, Subscriber
from applevision_rospkg.msg import PointWithCovarianceStamped, RegionOfInterestWithCovarianceStamped
from applevision_rospkg.srv import Tf2Transform
from applevision_kalman.filter import KalmanFilter, EnvProperties
from helpers import RobustServiceProxy, ServiceProxyFailed, HeaderCalc, CameraInfoHelper


def second_order_var_approx(fprime, fdoubleprime, ftrippleprime, xvar):
    # https://en.wikipedia.org/wiki/Taylor_expansions_for_the_moments_of_functions_of_random_variables
    return fprime**2 * xvar + 1 / 2 * fdoubleprime**2 * xvar**2 + fprime * ftrippleprime * xvar**2


class MainHandler:

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

    def callback(self, dist: Range,
                 cam: RegionOfInterestWithCovarianceStamped):
        self.cam_info.wait_for_camera_info()
        cam_info_obj = self.cam_info.get_last_camera_info()
        cam_res = np.array([cam_info_obj.width, cam_info_obj.height])
        cam_focal = np.array([cam_info_obj.P[0], cam_info_obj.P[5]])

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
        range_corrected = dist.range + self.kal.env.apple_r

        # publish the kalman filters predicted apple distance
        # if bounding box is on the edge the variance is infinite
        if cam.w == 0 or cam.h == 0 or cam.x == 0 or cam.x + cam.w == cam_res[
                0]:
            x_est, p_est, var = self.kal.step_filter(
                (0, 0, 0, range_corrected), np.inf, np.inf, np.inf, control)
        else:
            # compute apple x, y based off of the bounding box width/height avg
            # TODO: how to fix this? it's unreliable
            # TODO: improve varience calculations
            # TODO: initialize robot in correct position
            # TODO: this is wrong
            est_z = cam_focal[0] * (2 * self.kal.env.apple_r) / cam.w
            center_x = cam.x + cam.w / 2 - cam_res[0] / 2
            center_y = cam.y + cam.h / 2 - cam_res[1] / 2
            est_x = center_x * est_z / cam_focal[0]
            est_y = center_y * est_z / cam_focal[1]

            z_const = cam_focal[0] * (2 * self.kal.env.apple_r)
            z_fprime = z_const * -1 / cam.w**2
            z_fdoubleprime = z_const * 2 / cam.w**3
            z_ftrippleprime = z_const * -6 / cam.w**4
            z_var = second_order_var_approx(z_fprime, z_fdoubleprime,
                                            z_ftrippleprime, cam.w_var)

            # assume x and w uncorrelated (probably not true)
            center_x_var = cam.x_var + 1 / 4 * cam.w_var
            center_y_var = cam.x_var + 1 / 4 * cam.w_var
            # https://stats.stackexchange.com/questions/52646/variance-of-product-of-multiple-independent-random-variables
            x_var = ((center_x_var + center_x**2) * (z_var + est_z**2) -
                     (center_x * est_z)**2) * 1 / cam_focal[0]**2
            y_var = ((center_y_var + center_y**2) * (z_var + est_z**2) -
                     (center_y * est_z)**2) * 1 / cam_focal[0]**2

            # TODO: kalman filter will runaway sometimes?
            x_est, p_est, var = self.kal.step_filter(
                (est_x, est_y, est_z, range_corrected), x_var, y_var, z_var,
                control)

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
        out.camera_stamp = cam.header.stamp
        out.distance_stamp = dist.header.stamp
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

    # TODO: Tune these
    env = EnvProperties(delta_t_ms=33,
                        accel_std=0.005,
                        starting_position=(0, 0, 1),
                        starting_std=.4,
                        z_std=.005,
                        backdrop_dist=.5,
                        apple_r=.040,
                        dist_fov_rad=np.deg2rad(20))
    kal_filter = KalmanFilter(env, 1.5, 0.3, 0.75, 0.9)
    rospy.logdebug(
        f'Kalman filter using environment {env} and filter {kal_filter}.')

    # input('Press any key to start Applevision...')
    main_proc = MainHandler('applevision/est_apple_pos', kal_filter)
    dist = Subscriber('applevision/apple_dist', Range)
    camera = Subscriber('applevision/apple_camera',
                        RegionOfInterestWithCovarianceStamped)
    sync = ApproximateTimeSynchronizer([dist, camera], 10, 0.017)
    sync.registerCallback(main_proc.callback)

    rospy.spin()


if __name__ == '__main__':
    main()
