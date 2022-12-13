import csv
import numpy as np
import matplotlib as mpl
import matplotlib.image as mpimg
import transformations as tf

def in2m(inches):
    return 2.54 * inches / 100.0


def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)


def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)


def get_x(M):
    return M[:, 0, 3]


def set_x(M, x):
    M[:, 0, 3] = x


def get_y(M):
    return M[:, 1, 3]


def set_y(M, y):
    M[:, 1, 3] = y


def set_yaw(M, theta):
    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)
    M[:, 0, 0] = cos_theta
    M[:, 0, 1] = -sin_theta
    M[:, 1, 0] = sin_theta
    M[:, 1, 1] = cos_theta


def plot_field(ax):
    LENGTH = 15.98  # m
    WIDTH = 8.21  # m
    img = mpimg.imread('field.png')
    ax.imshow(img, extent=[-LENGTH * 0.5, LENGTH * 0.5, -WIDTH * 0.5, WIDTH * 0.5])


# Load and run a QuickPlan Trajectory
def load_traj(filename):
    traj = []
    with open(filename) as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            traj.append([float(r) for r in row])
    return np.array(traj)


# Using unicycle model for now
# f(x, u)
# x = x, y, theta
# u = v, w
# returns dx/dt
def dynamics_model(x, u):
    return np.array([
        u[0] * np.cos(x[2]),
        u[0] * np.sin(x[2]),
        u[1]
    ])


def besph2cart(bearing: float, elevation: float):
    """Converts a bearing and elevation in spherical coordinates to a 3D cartesian point on the unit circle."""
    return np.array(
        [
            np.cos(bearing) * np.cos(elevation),
            np.sin(bearing) * np.cos(elevation),
            np.sin(elevation),
        ]
    )


def cart2besph(x: float, y: float, z: float):
    """Converts the direction of a 3D cartesian point to a bearing and elevation in spherical coordinates."""
    return np.hstack((np.arctan2(y, x), np.arctan2(z, np.sqrt(np.power(x, 2) + np.power(y, 2)))))


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


def plot_robot(ax, pose, robot_axis_plot_size=0.1):
    # Plot robot axes
    ax.add_collection(
        mpl.collections.LineCollection(
            transform_geometry([[(0, 0), (robot_axis_plot_size, 0)]], pose), color="r",
        )
    )
    ax.add_collection(
        mpl.collections.LineCollection(
            transform_geometry([[(0, 0), (0, robot_axis_plot_size)]], pose), color="g",
        )
    )


def get_point_in_frame(frame, point):
    # This expression is inefficient due to the inverse: np.matmul(np.linalg.inv(frame), point)[:, :3]
    # Compute without using inverse by using the following:
    # inv(A) * [x] = [ inv(M) * (x - b) ]
    #          [1] = [        1         ]
    # where:
    # A = [ M   b  ]
    #     [ 0   1  ]
    # Also, note that the inverse of an affine rotation matrix is its transpose.
    assert len(frame.shape) == len(point.shape)

    if len(frame.shape) == 2:
        M = frame[:3, :3]
        M_inv = np.transpose(M)
        b = frame[:3, 3:4]
    elif len(frame.shape) == 3:
        M = frame[:, :3, :3]
        M_inv = np.transpose(M, axes=(0, 2, 1))
        b = frame[:, :3, 3:4]
    else:
        raise
    return np.matmul(M_inv, point - b)
