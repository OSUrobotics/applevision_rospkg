from typing import Tuple
import itertools
import numpy as np
from numpy.core.fromnumeric import clip

Vector2 = Tuple[float, float]
Vector3 = Tuple[float, float, float]


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


class ConeSensorModel:
    FOV_RAD = np.deg2rad(20)

    def __init__(self, backdrop_dist: float, apple_rad: float, stddev: float,
                 rng: np.random.Generator):
        self.backdrop_dist = backdrop_dist
        self.apple_rad = apple_rad
        self.stddev = stddev
        self.rng = rng

    def calc_per_apple_in_fov(self, x, y, z) -> float:
        '''calculate the percantage of the FOV which contains apple'''
        fov_at_apple_rad = z * np.tan(ConeSensorModel.FOV_RAD / 2)
        try:
            area_apple_in_fov = area_of_intersecting_circles(self.apple_rad, fov_at_apple_rad,
                                                             np.sqrt(x**2 + y**2))
        except ValueError:
            return 0
        return clip(area_apple_in_fov / (np.pi * fov_at_apple_rad**2), 0, 1)

    def measure(self, apple_pos: Vector3) -> float:
        rx, ry, rz = apple_pos
        # treat rz in terms of the apples surface
        rz -= self.apple_rad

        # calculate the percantage of the FOV which contains apple
        per_apple_in_fov = self.calc_per_apple_in_fov(rx, ry, rz)

        # print(f'Apple fov: {per_apple_in_fov*100:.2f}%')
        # > 80% means the apple is basically correct
        if per_apple_in_fov >= .8:
            mz = rz

        # < 80% means we get a linear interpolation between the apples center
        # and the backdrop
        elif per_apple_in_fov > 0:
            per_blur = .8 - per_apple_in_fov
            mz = rz * (1 - per_blur) + self.backdrop_dist * per_blur
        else:
            mz = self.backdrop_dist

        return max(0, mz + self.rng.normal(0, self.stddev))


class NormalCameraModel:

    def __init__(self, camera_dev: float, rng: np.random.Generator):
        self.camera_dev = camera_dev
        self.rng = rng

    def measure(self, apple_pos: Vector3) -> Tuple[Vector2, Vector2]:
        rx, ry, _ = apple_pos
        x = rx + self.rng.normal(0, self.camera_dev)
        y = ry + self.rng.normal(0, self.camera_dev)
        return (x, y), (self.camera_dev**2, self.camera_dev**2)


class AppleModel:

    def __init__(self, initial_pos: Vector3, speed: float, delta_t_ms: int,
                 camera: NormalCameraModel, sensor: ConeSensorModel):
        self.pos_real = initial_pos
        self.speed = speed
        self.delta_t_ms = delta_t_ms
        self.cur_time = 0
        self.camera = camera
        self.sensor = sensor
        self._last_velocity = np.array((0, 0, 0))

    def step(self) -> Tuple[Vector3, Vector2, np.ndarray]:
        delta_t = self.delta_t_ms / 1000
        if self.cur_time != 0:
            # move the robot towards the apple at a fixed speed
            new_velocity = (self.pos_real / np.linalg.norm(self.pos_real)) * -self.speed
            self.pos_real = self.pos_real + new_velocity * delta_t
        else:
            new_velocity = np.array((0, 0, 0))

        # compute control vector
        new_accel = (new_velocity - self._last_velocity) / delta_t
        control = np.transpose(
            np.array([
                new_velocity[0], new_accel[0], new_velocity[1], new_accel[1], new_velocity[2],
                new_accel[2]
            ]))
        self._last_velocity = new_velocity

        # step time
        self.cur_time += self.delta_t_ms / 1000

        # take a measurement
        campos, camvar = self.camera.measure(self.pos_real)
        distpos = self.sensor.measure(self.pos_real)

        return (campos[0], campos[1], distpos), camvar, control
