import csv
import numpy as np

def in2m(inches):
    # Inches to meters
    return inches * 2.54 / 100.0


def rotate_around_origin(point, theta):
    x, y = point
    return (
        x * np.cos(theta) - y * np.sin(theta),
        y * np.cos(theta) + x * np.sin(theta),
    )


def transform_geometry(geometry, pose):
    x, y, theta = pose
    transformed_geometry = []
    for point1, point2 in geometry:
        new_point1 = rotate_around_origin(point1, theta) + np.array([x, y])
        new_point2 = rotate_around_origin(point2, theta) + np.array([x, y])
        transformed_geometry.append((new_point1, new_point2))
    return transformed_geometry

def interp_state_vector(times, states, new_dt):
    interp_times = np.arange(0, times[-1], new_dt)
    return interp_times, np.interp(interp_times, times, states)

def write_to_csv(traj, filename):
    with open("./{}".format(filename), "w", newline='') as outfile:
        writer = csv.writer(outfile, delimiter=",")
        writer.writerows(traj)
    outfile.close()
