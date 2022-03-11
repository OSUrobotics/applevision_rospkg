#!/usr/bin/env python3
import rospy
import tf2_msgs.msg
from geometry_msgs.msg import TransformStamped


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


def main():
    rospy.init_node('applevision_publish_frames')

    pub_tf = rospy.Publisher("/tf_static", tf2_msgs.msg.TFMessage, queue_size=1, latch=True)
    # TODO: replace with real values
    now = rospy.Time.now()
    # frames are looking out from the robot arm with y pointed at target and z pointed up
    # grabber_tf = make_tf(now, 'tool0', 'fake_grabber', 0, 0.01, 0, ry=-0.7071, rz=-0.7071, rw=0)
    grabber_tf = make_tf(now, 'tool0', 'fake_grabber', 0, 0, 0)
    dist_tf = make_tf(now, 'fake_grabber', 'fake_grabber_dist', 0, 0, 0, ry=-0.7071, rw=0.7071)
    cam_tf = make_tf(now, 'fake_grabber', 'fake_grabber_cam', 0, 0, 0, rz=1, rw=0)
    apple_tf = make_tf(now, 'base_link', 'fake_apple', 0, 0.7, 0.5, ry=0.7071, rz=0.7071, rw=0)
    start_tf = make_tf(now, 'fake_apple', 'start_pos', 0, 0, -0.7)
    pub_tf.publish((grabber_tf, dist_tf, cam_tf, apple_tf, start_tf))

    rospy.loginfo('Publishing frames...')
    rospy.spin()


if __name__ == '__main__':
    main()
