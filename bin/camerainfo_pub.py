#!/usr/bin/python3

"""
RViz CameraPub doesn't publish camera info data.
This node is a work-around that will read in a camera calibration .yaml
file, convert it to a valid sensor_msgs/CameraInfo message, and publish it on a
topic.

Modified from https://gist.github.com/rossbar/ebb282c3b73c41c1404123de6cea4771.
"""
import sys
from pathlib import Path
import rospy
import yaml
from sensor_msgs.msg import CameraInfo


def yaml_to_CameraInfo(yaml_fname):
    """
    Parse a yaml file containing camera calibration data (as produced by
    rosrun camera_calibration cameracalibrator.py) into a
    sensor_msgs/CameraInfo msg.

    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data
    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
    """
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle, Loader=yaml.SafeLoader)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = calib_data["frame_id"]
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg


if __name__ == "__main__":
    args = rospy.myargv(sys.argv)

    # Get fname from command line (cmd line input required)
    import argparse
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument(
        "filename", type=Path,
        help="Path to yaml file containing camera calibration data")
    args = arg_parser.parse_args(args[1:])
    filename = args.filename

    # Parse yaml file
    camera_info_msg = yaml_to_CameraInfo(filename)

    # Initialize publisher node
    rospy.init_node("camera_info_publisher", anonymous=True)
    publisher = rospy.Publisher(f"{rospy.get_name()}/camera_info", CameraInfo, queue_size=10)
    rate = rospy.Rate(10)

    # Run publisher
    while not rospy.is_shutdown():
        camera_info_msg.header.seq += 1
        camera_info_msg.header.stamp = rospy.Time.now()
        publisher.publish(camera_info_msg)
        rate.sleep()
