#!/usr/bin/python3

import time
import rospy
import numpy as np
from sensor_msgs.msg import Range
from geometry_msgs.msg import TransformStamped
from applevision_rospkg.srv import Tf2Transform
from applevision_kalman.model import ConeSensorModel
from helpers import RobustServiceProxy, ServiceProxyFailed, HeaderCalc

APPLE_R = 0.04


class DistPub:

    def __init__(self, frame_id: str, topic: str) -> None:
        self.pub = rospy.Publisher(topic, Range, queue_size=10)
        self.tf_get = RobustServiceProxy('Tf2Transform',
                                         Tf2Transform,
                                         persistent=True)
        self._header_calc = HeaderCalc(frame_id)
        self._cone_sensor_model = ConeSensorModel(1.5, APPLE_R, 0.001,
                                                  np.random.default_rng())

    def callback(self, *args):
        # compute a fake distance based off of the robots position
        # TODO: drop random data points
        try:
            dist_to_apple = self.tf_get('applevision_target', 'palm_dist',
                                        rospy.Time(), rospy.Duration())
        except ServiceProxyFailed as e:
            rospy.logwarn(f'tf_get service proxy failed with error {e}')
            return
        trans: TransformStamped = dist_to_apple.transform
        # in this case x is the distance range, and z=-x in the robot coordinate space
        vect = (trans.transform.translation.z, trans.transform.translation.y,
                trans.transform.translation.x)
        dist = self._cone_sensor_model.measure(vect)
        self.pub.publish(header=self._header_calc.get_header(),
                         radiation_type=Range.INFRARED,
                         field_of_view=ConeSensorModel.FOV_RAD,
                         min_range=0,
                         max_range=1300,
                         range=dist)


def main():
    rospy.init_node('applevision_fake_sensor_data')
    rospy.wait_for_service('apply_planning_scene')
    rospy.wait_for_service('Tf2Transform')

    rospy.loginfo('Starting fake sensor data...')
    rospy.wait_for_service('Tf2Transform')
    dist_pub = DistPub('palm_dist', 'applevision/apple_dist')
    dist_timer = rospy.Timer(rospy.Duration.from_sec(1 / 32),
                             dist_pub.callback)

    while not rospy.is_shutdown():
        time.sleep(0.1)

    dist_timer.shutdown()


if __name__ == '__main__':
    main()
