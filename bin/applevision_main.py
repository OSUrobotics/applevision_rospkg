#!/usr/bin/python3

import rospy
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
        now = rospy.get_rostime()
        fake_header = Header(seq=self._seq, stamp=now, frame_id=self.frame_id)
        if self._seq >= 4294967295:  # uint32 max
            self._seq = 0
        else:
            self._seq += 1

        return fake_header

class MainHandler:
    def __init__(self, tf_get: rospy.ServiceProxy, p_out: rospy.Publisher, tf2_out: rospy.Publisher, kal: KalmanFilter) -> None:
        self.tf_get = tf_get
        self.p_out = p_out
        self.tf2_out = tf2_out
        self.kal = kal
        self._header = HeaderCalc('fake_grabber')
        self._gen = np.random.default_rng()

    def callback(self, dist: Range, cam: RegionOfInterestWithCovarianceStamped):
        # TODO: incorperate real kalman fiter
        # publish real apple distance with some noise
        dist_to_apple = self.tf_get('fake_apple', 'fake_grabber', rospy.Time.now()-rospy.Duration.from_sec(0.1), rospy.Duration.from_sec(5))
        trans: TransformStamped = dist_to_apple.transform
        trans.child_frame_id = 'apple'
        trans.transform.translation.x += self._gen.normal(0, 0.01)
        trans.transform.translation.y += self._gen.normal(0, 0.01)
        trans.transform.translation.z += self._gen.normal(0, 0.01)

        self.tf2_out.publish([trans])
        out = PointWithCovarianceStamped()
        out.header = self._header.get_header()
        out.camera_stamp = cam.header.stamp
        out.distance_stamp = dist.header.stamp
        out.point = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
        out.covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.p_out.publish(out)


def main():
    rospy.init_node('applevision_main')
    rospy.wait_for_service('Tf2Transform')

    # TODO: Tune these
    env = EnvProperties(delta_t_ms=33,
                        accel_std=1,
                        starting_position=(0, 0, 1000),
                        starting_std=400,
                        z_std=5,
                        backdrop_dist_mm=500,
                        apple_r_mm=80,
                        dist_fov_rad=np.deg2rad(25))
    kal_filter = KalmanFilter(env, 1.5, 0.75, 0.9)
    rospy.logdebug(f'Kalman filter using environment {env} and filter {kal_filter}.')

    # input('Press any key to start Applevision...')
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
