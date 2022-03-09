from typing import Tuple
import itertools

import numpy as np
from numpy.core.fromnumeric import clip

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


class EnvProperties:

    def __init__(self, delta_t_ms: int, accel_std: float, starting_position: Vec3,
                 starting_std: float, z_std: float, backdrop_dist: float, apple_r: float,
                 dist_fov_rad: float) -> None:
        self.delta_t_ms = delta_t_ms
        self.delta_t = delta_t_ms / 1000
        self.accel_std = accel_std
        self.starting_position = starting_position
        self.starting_std = starting_std
        self.z_std = z_std
        self.backdrop_dist = backdrop_dist
        self.apple_r = apple_r
        self.dist_fov_rad = dist_fov_rad


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
        F = np.eye(3)
        G = np.eye(3)
        Q = env.accel_std**2 * G @ np.transpose(G)
        H = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
            [0, 0, 1]
        ])
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

    def __init__(self, env: EnvProperties, std_range: float, appl_proportion_low: float,
                 appl_proportion_high: float) -> None:
        self.env = env
        self.std_range = std_range
        self.appl_low = appl_proportion_low
        self.appl_high = appl_proportion_high

        self.mtx = KalmanMatricies.make_for_robot_model(env)
        self.x_est = np.transpose(env.starting_position)
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

    def _compute_var(self, mes_pos: Tuple, varx: float, vary: float, varz: float, est_pos: Vec3) -> float:
        mx, my, mzc, mzd = mes_pos
        ex, ey, ez = est_pos
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

        # compute predicted varience
        if min(all_pos_per_apple_in_fov) < self.appl_low:
            # this measurement is unlikely to be accurate, therefore our variance is maximum
            # varz = self.env.backdrop_dist**2
            varz_dist = np.inf
            covzcx = 0
            covzcy = 0
        else:
            mes_per_apple_in_fov = KalmanFilter._calc_per_apple_in_fov(self.env, (mx, my, ez))
            if mes_per_apple_in_fov >= self.appl_high:
                varz_dist = self.env.z_std**2 + (self.env.apple_r / 2)**2
                covzcx = 0
                covzcy = 0
            else:
                expected_fov_at_apple_rad = ez * np.tan(self.env.dist_fov_rad / 2)
                expected_per_apple_in_fov = self._calc_per_apple_in_fov(self.env, (ex, ey, ez))

                d_var_est = varx + vary
                a_var_est = (1 / 4 + (1 / (2 * self.env.apple_r))**2 +
                             (1 / (2 * expected_fov_at_apple_rad))**2) * d_var_est
                varz_dist = (self.env.backdrop_dist * (1 - expected_per_apple_in_fov))**2 + max(
                    self.env.backdrop_dist**2 * a_var_est**2, 0)

                eacx = np.pi * ex - (1 / 2 + 1 / (2 * self.env.apple_r) + 1 /
                                     (2 * expected_fov_at_apple_rad)) * (varx + ex**2 + ey)
                covzcx = self.env.backdrop_dist * eacx
                eacy = np.pi * ey - (1 / 2 + 1 / (2 * self.env.apple_r) + 1 /
                                     (2 * expected_fov_at_apple_rad)) * (vary + ex + ey**2)
                covzcy = self.env.backdrop_dist * eacy
        # TODO: covarience is broken
        var = np.array([
            [varx, 0, 0, 0],
            [0, vary, 0,  0],
            [0, 0, varz, 0],
            [0, 0, 0, varz_dist]])
        return np.maximum(var, 0)

    def step_filter(self, meas_pos: Tuple, varx: float, vary: float, varz: float,
                    control: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        meas = np.transpose(meas_pos)
        # this matrix may have infinity
        var = self._compute_var(meas_pos, varx, vary, varz, np.transpose(self.x_est))

        # TODO: improve control matrix handling
        if self.last_ctrl is None:
            x_predict = self.mtx.F @ self.x_est
        else:
            x_predict = self.mtx.F @ self.x_est + self.mtx.G @ (control - self.last_ctrl)
        self.last_ctrl = control
        p_predict = self.mtx.F @ self.p_est @ np.transpose(self.mtx.F) + self.mtx.Q
        K = p_predict @ np.transpose(
            self.mtx.H) @ np.linalg.inv(self.mtx.H @ p_predict @ np.transpose(self.mtx.H) + var)
        self.x_est = x_predict + K @ (meas - self.mtx.H @ x_predict)
        fact = (self.mtx.I - K @ self.mtx.H)
        # replace inf with a very large number to prevent nans being generated when inf*0 occurs
        var = np.nan_to_num(var, nan=0, posinf=1e9)
        self.p_est = fact @ p_predict @ np.transpose(fact) + K @ var @ np.transpose(K)
        self.p_est = np.maximum(self.p_est, 0)

        assert np.all(var >= 0)

        return self.x_est, self.p_est, var
