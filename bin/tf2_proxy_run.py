#!/usr/bin/python

# tf2_ros requires python2, so this file exposes a service that returns
# the robots position.

from __future__ import print_function

from applevision_rospkg.srv import Tf2Transform
from threading import Lock
import rospy
import tf2_ros


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

    rospy.loginfo('Starting Tf2 server...')
    s = rospy.Service('Tf2Transform', Tf2Transform, get_transform)
    rospy.spin()


if __name__ == '__main__':
    main()
