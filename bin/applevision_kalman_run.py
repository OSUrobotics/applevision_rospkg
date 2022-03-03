#!/usr/bin/python3

import rospy
from std_msgs.msg import UInt16, Float64MultiArray
import numpy as np
from pathlib import Path
from applevision_kalman_pkg.filter import KalmanFilter, EnvProperties

PATH_TO_COORDS = Path('coords.txt')


def read_coordinates(path_to_coords: Path):
    # format is x, y, z, xvar, yvar, control0 .. control6
    text = path_to_coords.read_text().strip()
    return [float(val.strip()) for val in text.split(',')]


def main(path_to_coords: Path):
    rospy.init_node('listener', anonymous=True)

    env = EnvProperties(delta_t_ms=33,
                        accel_std=1,
                        starting_position=(0, 0, 1000),
                        starting_std=400,
                        z_std=5,
                        backdrop_dist_mm=500,
                        apple_r_mm=80,
                        dist_fov_rad=np.deg2rad(25))
    kal_filter = KalmanFilter(env, 1.5, 0.75, 0.9)

    p = rospy.Publisher('kalman_out', Float64MultiArray, queue_size=10)

    def handle_invoke(data):
        appl_dist = data.data
        coords = read_coordinates(path_to_coords)
        x_est, p_est, _ = kal_filter.step_filter(coords[0:2] + [appl_dist],
                                                 coords[3], coords[4],
                                                 np.transpose(coords[5:]))

        send = [x for x in x_est.flat] + [p for p in p_est.flat]
        p.publish(data=send)

    s = rospy.Subscriber('appl_dist', UInt16, handle_invoke)

    rospy.spin()


if __name__ == '__main__':
    main(PATH_TO_COORDS)
