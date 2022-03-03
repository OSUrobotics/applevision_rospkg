import numpy as np
import matplotlib as mpl

mpl.use('TkAgg')  # ignore
from matplotlib import pyplot as plt

from applevision_kalman.model import AppleModel, NormalCameraModel, ConeSensorModel
from applevision_kalman.filter import KalmanFilter, EnvProperties


def plot_state(num, title, time_log, real_log, est_log, meas_log):
    fig1 = plt.figure()
    fig1.suptitle(title)
    real, = plt.plot(time_log, [m[num] for m in real_log])
    est_z, *_ = plt.errorbar(time_log, [est[num] for est, _ in est_log],
                             [np.sqrt(var[num, num]) for meas, var in est_log])
    meas, *_ = plt.errorbar(time_log, [m[num] for m, _ in meas_log],
                            [np.sqrt(var[num, num]) for meas, var in meas_log])
    plt.legend([real, est_z, meas], ('Real', 'Estimated', 'Measured'))


if __name__ == '__main__':
    np.seterr(all='raise')
    ITERATIONS = 80

    env = EnvProperties(delta_t_ms=33,
                        accel_std=0,
                        starting_position=(0, 0, 1000),
                        starting_std=400,
                        z_std=5,
                        backdrop_dist_mm=500,
                        apple_r_mm=80,
                        dist_fov_rad=np.deg2rad(25))
    kal_filter = KalmanFilter(env, 1.5, 0.75, 0.9)

    rng = np.random.default_rng()
    model = AppleModel((0, 0, 800), 500, env.delta_t_ms, NormalCameraModel(0.01, rng),
                       ConeSensorModel(500, 80, 5, rng))

    meas_log = []
    est_log = []
    real_log = []
    time_log = []

    for _ in range(ITERATIONS):
        meas, (varx, vary), control = model.step()
        x_est, p_est, var = kal_filter.step_filter(meas, varx, vary, control)

        est_log.append((x_est, p_est))
        meas_log.append((meas, var))
        real_log.append(model.pos_real)
        time_log.append(model.cur_time)

    plot_state(0, 'x', time_log, real_log, est_log, meas_log)
    plot_state(1, 'y', time_log, real_log, est_log, meas_log)
    plot_state(2, 'z', time_log, real_log, est_log, meas_log)

    plt.show(block=True)
