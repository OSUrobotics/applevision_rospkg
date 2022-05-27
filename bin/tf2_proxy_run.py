#!/usr/bin/python

# tf2_ros requires python2, so this file exposes a service that returns
# the robots position.

from __future__ import print_function

from applevision_rospkg.srv import Tf2Transform, Tf2TransformPoseStamped
from threading import Lock
import rospy
import tf2_ros
import tf2_geometry_msgs


def main():
    rospy.init_node('Tf2Server')
    tf_buffer_lock = Lock()
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    def get_transform(req):
        with tf_buffer_lock:
            try:
                return tf_buffer.lookup_transform(req.target_frame, req.source_frame, req.stamp, req.slop)
            except Exception as e:
                raise rospy.ServiceException(e)

    def transform_pose(req):
        with tf_buffer_lock:
            try:
                return tf_buffer.transform(req.pose, req.target_frame, req.timeout)
            except Exception as e:
                raise rospy.ServiceException(e)

    rospy.loginfo('Starting Tf2 server...')
    s1 = rospy.Service('Tf2Transform', Tf2Transform, get_transform)
    s2 = rospy.Service('Tf2TransformPoseStamped', Tf2TransformPoseStamped, transform_pose)
    rospy.spin()


if __name__ == '__main__':
    main()
