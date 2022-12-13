import matplotlib.pyplot as plt
from multiprocessing import Process
import numpy as np
import time
import transformations as tf

from data_utils import Odometry, Vision
from helpers import cart2besph, get_point_in_frame, plot_field, load_traj
from particle_filter import ParticleFilter, ROBOT_TO_CAMERA, get_point_on_target_circle
from plotter import Plotter


def run_traj(num_particles, plotter_queue):
    traj = load_traj("./trajectories/drive_everywhere.csv")

    fig, ax = plt.subplots()
    plt.axis("equal")
    plot_field(ax)

    pf = ParticleFilter(num_particles, Odometry(0, traj[0, 1], traj[0, 2], traj[0, 3]))
    noisy_T = tf.compose_matrix(
        translate=[traj[0, 1], traj[0, 2], 0], angles=[0, 0, traj[0, 3]]
    )
    # pf.plot_particles(plt)
    # pf.plot_estimate(plt)

    for i in range(1, traj.shape[0]):
        print(f"------ Timestep: {i}")

        x, y, theta = traj[i, 1:4]
        last_x, last_y, last_theta = traj[i - 1, 1:4]

        # Compute deltas
        T = tf.compose_matrix(translate=[x, y, 0], angles=[0, 0, theta])
        last_T = tf.compose_matrix(
            translate=[last_x, last_y, 0], angles=[0, 0, last_theta]
        )
        M = np.linalg.inv(last_T).dot(T)
        _, _, angles, trans, _ = tf.decompose_matrix(M)
        dx, dy, _ = trans
        _, _, dtheta = angles

        # Simulate noisy deltas
        xNoise = 3e-1 * abs(dx)  # Scale uncertainty by distance
        yNoise = 3e-1 * abs(dy)
        thetaNoise = 1e-6
        dx += np.random.normal(0.0, xNoise)
        dy += np.random.normal(0.0, yNoise)
        dtheta += np.random.normal(0.0, thetaNoise)
        noisy_T = noisy_T.dot(
            tf.compose_matrix(translate=[dx, dy, 0], angles=[0, 0, dtheta])
        )
        _, _, noisy_angles, noisy_trans, _ = tf.decompose_matrix(noisy_T)
        noisy_x, noisy_y, _ = noisy_trans
        _, _, noisy_theta = noisy_angles

        # Do predict
        s = time.perf_counter()
        pf.predict(Odometry(i, noisy_x, noisy_y, noisy_theta))
        print(f"Predict: {time.perf_counter() - s}")

        # Simulate measurements
        target_direction_mod = np.mod(np.arctan2(y, x) + np.pi, 2 * np.pi)
        theta_mod = np.mod(theta, 2.0 * np.pi)
        measurements = []
        if np.pi - abs(abs(target_direction_mod - theta_mod) - np.pi) < np.deg2rad(30):
            T = tf.compose_matrix(translate=[x, y, 0], angles=[0, 0, theta])
            robot_to_camera_T = T.dot(ROBOT_TO_CAMERA)
            point_on_target_circle = get_point_on_target_circle(
                robot_to_camera_T[0, 3], robot_to_camera_T[1, 3]
            )
            mx, my, mz = get_point_in_frame(
                robot_to_camera_T, point_on_target_circle[0]
            )
            be, ev = cart2besph(mx, my, mz)
            measurements.append(Vision(id, be, ev))

        # Do update & resample
        s = time.perf_counter()
        has_vision = pf.update(measurements)
        print(f"Update & resample (vision: {has_vision}): {time.perf_counter() - s}")

        pf.plot_particles(plt, has_vision)
        plotter_queue.put((pf.particles, pf.get_best_estimate(), has_vision))

    plt.plot(traj[:, 1], traj[:, 2])
    plt.show()


if __name__ == "__main__":
    NUM_PARTICLES = 100000
    # Start plotter thread
    plotter = Plotter(NUM_PARTICLES)
    plotter_queue = plotter.start()

    # Start particle filtler thread
    t = Process(
        target=run_traj,
        args=(
            NUM_PARTICLES,
            plotter_queue,
        ),
    )
    t.start()

    print("Waiting for IO thread to join...")
    t.join()
    print("Waiting for graph window process to join...")
    plotter.join()
