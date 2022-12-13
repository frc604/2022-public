from turtle import position
import matplotlib.pyplot as plt
from multiprocessing import Process
from networktables import NetworkTables
import numpy as np
import time
import transformations as tf

from helpers import cart2besph, get_point_in_frame, plot_field, load_traj
from particle_filter import ROBOT_TO_CAMERA, get_point_on_target_circle
from quixsam import Quixsam


KEY = "0"


def mock_robot():
    NetworkTables.initialize()
    quixsam_table = NetworkTables.getTable("quixsam")
    odometry_table = quixsam_table.getSubTable("odometry")
    vision_table = quixsam_table.getSubTable("vision")
    estimates_table = quixsam_table.getSubTable("estimates")

    # Setup estimates listener
    xs = []
    ys = []

    def estimate_received(table, key, value, is_new):
        _, x, y, _ = value
        xs.append(x)
        ys.append(y)

    estimates_table.addEntryListener(estimate_received)

    traj = load_traj("./trajectories/orbit.csv")
    for id in range(traj.shape[0]):
        # Mock odometry
        x, y, theta = traj[id, 1:4]
        odometry_table.putNumberArray(KEY, [id, x, y, theta, 0, 0, 0])

        # Mock vision
        target_direction_mod = np.mod(np.arctan2(y, x) + np.pi, 2 * np.pi)
        theta_mod = np.mod(theta, 2.0 * np.pi)

        # Check that we are facing the target within some FOV
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
            vision_array = [
                id,
                be,
                ev,
                0,
                0,
            ]
            vision_table.getEntry(KEY).setNumberArray(vision_array)

        NetworkTables.flush()
        time.sleep(0.02)

    # Plot result vs. ground truth
    _, ax = plt.subplots()
    plt.axis("equal")
    plot_field(ax)
    plt.plot(traj[:, 1], traj[:, 2])  # Ground truth
    plt.scatter(xs, ys, marker=".", color="r")  # Estimate
    plt.show()


def run_quixsam():
    Quixsam("localhost").run()


if __name__ == "__main__":
    # Start Quixsam
    qs = Process(target=run_quixsam)
    qs.start()

    # Start running mock robot
    p = Process(target=mock_robot)
    p.start()

    # Join threads
    qs.join()
    p.join()
