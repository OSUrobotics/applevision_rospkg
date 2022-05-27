from typing import Optional, Tuple
import itertools

import numpy as np
from numpy.core.fromnumeric import clip
from sensor_msgs.msg import CameraInfo, Range
from applevision_rospkg.msg import RegionOfInterestWithConfidenceStamped

# state variables: x, y, z
# control input: robot x, y, z from starting estimate point
# process noise: constant acceleration model
# sensors: x, y, z cam, z sensor
# units are m k s
# axis is right hand rule, positive y points towards apple, z=0 centered around apple

Vec3 = Tuple[float, float, float]


def area_of_intersecting_circles(rad_1, rad_2, center_dist):
    if center_dist <= max(rad_1, rad_2) - min(rad_1, rad_2):
        return np.pi * min(rad_1, rad_2)**2

    if center_dist == rad_1 + rad_2:
        return 0

    if center_dist > rad_1 + rad_2:
        raise ValueError("Cannot calculate the area of non-intersecting circles")

    part_1 = (rad_1**2) * np.arccos(
        (center_dist**2 + rad_1**2 - rad_2**2) / (2 * center_dist * rad_1))
    part_2 = (rad_2**2) * np.arccos(
        (center_dist**2 + rad_2**2 - rad_1**2) / (2 * center_dist * rad_2))
    part_3 = 0.5 * np.sqrt((-center_dist + rad_1 + rad_2) * (center_dist + rad_1 - rad_2) *
                           (center_dist - rad_1 + rad_2) * (center_dist + rad_1 + rad_1))
    return part_1 + part_2 - part_3


def second_order_var_approx(fprime, fdoubleprime, ftrippleprime, xvar):
    # https://en.wikipedia.org/wiki/Taylor_expansions_for_the_moments_of_functions_of_random_variables
    return fprime**2 * xvar + 1 / 2 * fdoubleprime**2 * xvar**2 + fprime * ftrippleprime * xvar**2


def to_column(ray) -> np.ndarray:
    return np.transpose(np.atleast_2d(ray))


class EnvProperties:

    def __init__(self, delta_t_ms: int, accel_std: float, starting_position: Vec3,
                 starting_std: float, z_std: float, backdrop_dist: float, apple_r: float,
                 dist_fov_rad: float, camera_info: CameraInfo) -> None:
        self.delta_t_ms = delta_t_ms
        self.delta_t = delta_t_ms / 1000
        self.accel_std = accel_std
        self.starting_position = starting_position
        self.starting_std = starting_std
        self.z_std = z_std
        self.backdrop_dist = backdrop_dist
        self.apple_r = apple_r
        self.dist_fov_rad = dist_fov_rad
        self.camera_info = camera_info


class KalmanMatricies:

    def __init__(self, F: np.ndarray, G: np.ndarray, Q: np.ndarray, H: np.ndarray,
                 I: np.ndarray) -> None:
        self.F = F
        self.G = G
        self.Q = Q
        self.H = H
        self.I = I

    @staticmethod
    def make_for_robot_model(env: EnvProperties) -> 'KalmanMatricies':
        t = env.delta_t
        F = np.eye(3)  # not used currently
        G = np.eye(3)
        Q = env.accel_std**2 * G @ np.transpose(G)
        H = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, 0, 1]])
        I = np.eye(H.shape[1], H.shape[1])

        return KalmanMatricies(F, G, Q, H, I)

    @staticmethod
    def _concat_optional(a, *args, **kwargs) -> np.ndarray:
        a = [i for i in a if i is not None]
        return np.concatenate(a, *args, **kwargs)

    @staticmethod
    def _upscale_dim(mtx: np.ndarray, ndim: int) -> np.ndarray:
        assert ndim > 0

        base = None
        for i in range(ndim):
            col = None
            for j in range(ndim):
                if i == j:
                    col = KalmanMatricies._concat_optional((col, mtx))
                else:
                    col = KalmanMatricies._concat_optional((col, np.zeros_like(mtx)))
            base = KalmanMatricies._concat_optional((base, col), axis=1)
        return base


class KalmanFilter():

    def __init__(self, env: EnvProperties, std_range: float, too_close_cam: float,
                too_far_dist: float, appl_proportion_low: float, appl_proportion_high: float) -> None:
        self.env = env
        self.std_range = std_range
        self.too_close_cam = too_close_cam
        self.too_far_dist = too_far_dist
        self.appl_low = appl_proportion_low
        self.appl_high = appl_proportion_high

        self.mtx = KalmanMatricies.make_for_robot_model(env)
        self.x_est_base = to_column(env.starting_position)
        self.p_est = np.eye(3) * self.env.starting_std**2

    @staticmethod
    def _calc_per_apple_in_fov(env: EnvProperties, pos: Vec3) -> float:
        '''calculate the percantage of the FOV which contains apple'''
        x, y, z = pos
        fov_at_apple_rad = z * np.tan(env.dist_fov_rad / 2)
        try:
            area_apple_in_fov = area_of_intersecting_circles(env.apple_r, fov_at_apple_rad,
                                                             np.sqrt(x**2 + y**2))
        except ValueError:
            return 0
        return clip(area_apple_in_fov / (np.pi * fov_at_apple_rad**2), 0, 1)

    @staticmethod
    def _compute_camera_from_bounding_box(env: EnvProperties, box: RegionOfInterestWithConfidenceStamped) -> Tuple[Vec3, Vec3]:
        cam_res = np.array([env.camera_info.width, env.camera_info.height])
        cam_focal = np.array([env.camera_info.P[0], env.camera_info.P[5]])
        cam_off = np.array([env.camera_info.P[2], env.camera_info.P[6]]) - cam_res/2

        # compute apple x, y based off of the bounding box width/height avg
        # TODO: how to fix this? it's unreliable
        # TODO: improve varience calculations
        ALL_PIX_VAR = 30**2
        est_z = cam_focal[0] * (2 * env.apple_r) / box.w
        center_x = box.x + box.w / 2 - cam_res[0] / 2
        center_y = box.y + box.h / 2 - cam_res[1] / 2
        # I think the center offsets approximate the offcenter camera sensor
        est_x = est_z * (center_x - cam_off[0]) / cam_focal[0]
        est_y = est_z * (center_y - cam_off[1]) / cam_focal[1]

        z_const = cam_focal[0] * (2 * env.apple_r)
        z_fprime = z_const * -1 / box.w**2
        z_fdoubleprime = z_const * 2 / box.w**3
        z_ftrippleprime = z_const * -6 / box.w**4
        z_var = second_order_var_approx(
            z_fprime, z_fdoubleprime, z_ftrippleprime, ALL_PIX_VAR)

        # assume x and w uncorrelated (probably not true)
        center_x_var = ALL_PIX_VAR + 1 / 4 * ALL_PIX_VAR
        center_y_var = ALL_PIX_VAR + 1 / 4 * ALL_PIX_VAR
        # https://stats.stackexchange.com/questions/52646/variance-of-product-of-multiple-independent-random-variables
        x_var = ((center_x_var + center_x**2) * (z_var + est_z**2) -
                    (center_x * est_z)**2) * 1 / cam_focal[0]**2
        y_var = ((center_y_var + center_y**2) * (z_var + est_z**2) -
                    (center_y * est_z)**2) * 1 / cam_focal[0]**2

        return (est_x, est_y, est_z), (x_var, y_var, z_var)

    def _compute_dist_var(self, cam_pos: Vec3, cam_var: Vec3, dist_meas: float, est_pos: Vec3) -> float:
        mx, my, mzc = cam_pos
        mzd = dist_meas
        varx, vary, varz = cam_var
        ex, ey, ez = est_pos

        # if any of varx, vary, varz is infinite or the distance is too close
        # for the camera, fallback to using the estimated position to compute
        # variance hope we've figured it out by now
        if np.any(np.isposinf((varx, vary, varz))):
            min_per_apple_in_fov = KalmanFilter._calc_per_apple_in_fov(self.env, (ex, ey, mzd))
        else:
            # determine if the apple is likely to be >80% of the FOV (we'll use 1.5sigma)
            x_err = np.sqrt(varx) * self.std_range
            minmax_x = (ex + x_err, ex - x_err, mx)
            y_err = np.sqrt(vary) * self.std_range
            minmax_y = (ey + y_err, ey - y_err, my)
            z_err = self.env.z_std * self.std_range
            all_pos_per_apple_in_fov = [
                KalmanFilter._calc_per_apple_in_fov(self.env, (x, y, mzd + z_err))
                for x, y in itertools.product(minmax_x, minmax_y)
            ]

            min_per_apple_in_fov = min(all_pos_per_apple_in_fov)

        # compute predicted varience
        if min_per_apple_in_fov < self.appl_low:
            # this measurement is unlikely to be accurate, therefore our variance is maximum
            # varz = self.env.backdrop_dist**2
            return np.inf
            # covzcx = 0
            # covzcy = 0

        mes_per_apple_in_fov = KalmanFilter._calc_per_apple_in_fov(self.env, (mx, my, ez))
        if mes_per_apple_in_fov >= self.appl_high:
            return self.env.z_std**2 + (self.env.apple_r / 2)**2
            # covzcx = 0
            # covzcy = 0

        expected_fov_at_apple_rad = ez * np.tan(self.env.dist_fov_rad / 2)
        expected_per_apple_in_fov = self._calc_per_apple_in_fov(self.env, (ex, ey, ez))

        d_var_est = varx + vary
        a_var_est = (1 / 4 + (1 / (2 * self.env.apple_r))**2 +
                        (1 / (2 * expected_fov_at_apple_rad))**2) * d_var_est
        return (self.env.backdrop_dist * (1 - expected_per_apple_in_fov))**2 + max(
            self.env.backdrop_dist**2 * a_var_est**2, 0)

        # eacx = np.pi * ex - (1 / 2 + 1 / (2 * self.env.apple_r) + 1 /
        #                     (2 * expected_fov_at_apple_rad)) * (varx + ex**2 + ey)
        # covzcx = self.env.backdrop_dist * eacx
        # eacy = np.pi * ey - (1 / 2 + 1 / (2 * self.env.apple_r) + 1 /
        #                     (2 * expected_fov_at_apple_rad)) * (vary + ex + ey**2)
        # covzcy = self.env.backdrop_dist * eacy
        # TODO: covarience is broken

    def step_filter(self, box: Optional[RegionOfInterestWithConfidenceStamped], dist: Optional[Range], control: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        # The kalman filter is with respect to a fixed starting point, but the
        # measurements are with respect to the end effector. These two
        # coordinate systems are resolved in the filter using some hacky
        # adding and subtracting of the offset between the two.
        ctrl_off = to_column(self.mtx.G @ control)
        x_predict_ee = self.x_est_base - ctrl_off

        dist_range = dist.range + self.env.apple_r if dist else 8
        # if bounding box is on the edge the variance is infinite
        if box is None or box.w == 0 or box.h == 0 or box.x == 0 or box.x + box.w == box.image_w or dist_range < self.too_close_cam:
            # TODO: larger edge? variance based on edge?
            # TODO: check for jumping boxes
            cam_var = (np.inf, np.inf, np.inf)
            cam_pos = (0, 0, 0)
        else:
            cam_pos, cam_var = self._compute_camera_from_bounding_box(self.env, box)

        # if distance sensor is improbable ignore
        if not dist or dist_range > self.too_far_dist:
            distvar = np.inf
        else:
            distvar = self._compute_dist_var(cam_pos, cam_var, dist.range, np.transpose(x_predict_ee)[0])

        # this matrix may have infinity
        var = np.maximum(0, np.array([
            [cam_var[0], 0, 0, 0],
            [0, cam_var[1], 0, 0],
            [0, 0, cam_var[2], 0],
            [0, 0, 0, distvar]]))
        meas = to_column(cam_pos + (dist_range,))

        p_predict = self.mtx.F @ self.p_est @ np.transpose(self.mtx.F) + self.mtx.Q
        K = p_predict @ np.transpose(
            self.mtx.H) @ np.linalg.inv(self.mtx.H @ p_predict @ np.transpose(self.mtx.H) + var)
        x_est_ee = x_predict_ee + K @ (meas - self.mtx.H @ x_predict_ee)
        self.x_est_base = x_est_ee + ctrl_off
        fact = (self.mtx.I - K @ self.mtx.H)
        # replace inf with a very large number to prevent nans being generated when inf*0 occurs
        var = np.nan_to_num(var, nan=0, posinf=1e9)
        self.p_est = fact @ p_predict @ np.transpose(fact) + K @ var @ np.transpose(K)
        self.p_est = np.maximum(self.p_est, 0)

        assert np.all(var >= 0)

        return np.transpose(x_est_ee)[0], self.p_est, var
