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
from helpers import RobustServiceProxy, ServiceProxyFailed, HeaderCalc


class MainHandler:
    CAMERA_RES = (640, 360)
    CAMERA_SENSOR = (5449*(640/672)*1e-6, 3072*(360/380)*1e-6)
    CAMERA_FOCAL = 11e-3

    def __init__(self, topic: str, kal: KalmanFilter) -> None:
        self.tf_get = RobustServiceProxy('Tf2Transform', Tf2Transform, persistent=True)
        self.p_out = rospy.Publisher(topic, PointWithCovarianceStamped, queue_size=10)
        self.viz_out = rospy.Publisher('applevision/est_apple_viz', PoseWithCovarianceStamped, queue_size=10)
        self.tf2_out = rospy.Publisher('tf', TFMessage, queue_size=10)
        self.kal = kal
        self._header = HeaderCalc('fake_grabber')
        self._gen = np.random.default_rng()

    def callback(self, dist: Range, cam: RegionOfInterestWithCovarianceStamped):
        try:
            dist_to_apple = self.tf_get('start_pos', 'fake_grabber', rospy.Time(), rospy.Duration())
        except ServiceProxyFailed as e:
            rospy.logwarn(f'tf_get service proxy failed with error {e}')
            return
        trans: TransformStamped = dist_to_apple.transform
        control = np.transpose(np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]))

        # publish the kalman filters predicted apple distance
        # if bounding box is on x edge, the variance is infinite
        if cam.w == 0 or cam.h == 0 or cam.x == 0 or cam.x + cam.w == self.CAMERA_RES[0]:
            x_est, p_est, var = self.kal.step_filter(
                (0, 0, 0, dist.range),
                np.inf,
                np.inf,
                np.inf,
                control)
        else:
            # compute apple x, y based off of the bounding box width/height avg
            # TODO: how to fix this? it's unreliable
            # TODO: improve varience calculations
            # TODO: initialize robot in correct position
            est_frame_width_m = self.CAMERA_RES[0]/cam.w*(2*self.kal.env.apple_r)
            est_frame_height_m = est_frame_width_m*(self.CAMERA_RES[1]/self.CAMERA_RES[0])
            center_x = cam.x + cam.w/2
            center_y = cam.y + cam.h/2
            est_x = -(center_x - self.CAMERA_RES[0]/2)/self.CAMERA_RES[0]*est_frame_width_m
            est_y = -(center_y - self.CAMERA_RES[1]/2)/self.CAMERA_RES[1]*est_frame_height_m
            est_z = (est_frame_width_m/2)*((2*self.CAMERA_FOCAL)/self.CAMERA_SENSOR[0])

            # frame_width_var_est = cam.w_var*(est_frame_width_m)**2
            # x_var = cam.x_var*(2*self.kal.env.apple_r/cam.w)**2 + cam.w_var*(2*self.kal.env.apple_r/cam.w**2)**2
            # y_var = cam.y_var*(2*self.kal.env.apple_r/cam.h)**2 + cam.h_var*(2*self.kal.env.apple_r/cam.h**2)**2
            frame_width_var_est = (1/cam.w_var**2)*(self.CAMERA_RES[0]*(2*self.kal.env.apple_r))**2
            x_var = (cam.x_var + cam.w_var*0.25 + frame_width_var_est)*(est_frame_width_m/self.CAMERA_RES[0])**2
            y_var = (cam.y_var + cam.h_var*0.25 + frame_width_var_est)*(est_frame_height_m/self.CAMERA_RES[1])**2
            # multiply by a process noise factor to tweak this estimation
            x_var *= 10
            y_var *= 10
            if cam.w == self.CAMERA_RES[0]:
                z_var = np.inf
            else:
                # TODO: z var is still broken
                z_var = frame_width_var_est*(0.5*((2*self.CAMERA_FOCAL)/self.CAMERA_SENSOR[0]))**2

            # TODO: kalman filter will runaway sometimes?
            x_est, p_est, var = self.kal.step_filter(
                (est_x, est_y, est_z, dist.range),
                x_var,
                y_var,
                z_var,
                control)

        trans_out = TransformStamped()
        trans_out.header = self._header.get_header()
        trans_out.header.frame_id = 'fake_grabber'
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

        out_viz = PoseWithCovarianceStamped()
        out_viz.header = self._header.get_header()
        out_viz.pose.pose.position = trans_out.transform.translation
        out_viz.pose.pose.orientation.w = 1
        out_viz.pose.covariance = np.pad(p_est, ((0, 3), (0, 3))).flatten().tolist()
        self.viz_out.publish(out_viz)



def main():
    rospy.init_node('applevision_main')
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
    kal_filter = KalmanFilter(env, 1.5, 0.75, 0.9)
    rospy.logdebug(f'Kalman filter using environment {env} and filter {kal_filter}.')

    # input('Press any key to start Applevision...')
    # TODO: service proxy is unreliable, needs a retry mechanism
    main_proc = MainHandler('applevision/est_apple_pos', kal_filter)
    dist = Subscriber('applevision/apple_dist', Range)
    camera = Subscriber('applevision/apple_camera', RegionOfInterestWithCovarianceStamped)
    sync = ApproximateTimeSynchronizer([dist, camera], 10, 0.017)
    sync.registerCallback(main_proc.callback)

    rospy.spin()


if __name__ == '__main__':
    main()
