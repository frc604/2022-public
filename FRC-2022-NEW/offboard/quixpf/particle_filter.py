from getopt import getopt
from typing import List
from helpers import (
    cart2pol,
    pol2cart,
    in2m,
    cart2besph,
    get_point_in_frame,
    get_x,
    set_x,
    get_y,
    set_y,
    set_yaw,
)
import numpy as np
import transformations as tf

from data_utils import Odometry, Vision, Landmark

# SIR Particle Filter

ROBOT_HALF_LENGTH = 0.5 * in2m(34)  # m
ROBOT_HALF_WIDTH = 0.5 * in2m(34)  # m
MIN_ROBOT_DIMENSION = min(ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH)

ROBOT_TO_LAUNCHER_PIVOT = tf.compose_matrix(
    translate=[in2m(2.0), 0, in2m(20.25)],
)
LAUNCHER_PIVOT_TO_LAUNCHER = tf.compose_matrix(
    angles=[0, np.deg2rad(5), 0],
)
LAUNCHER_TO_CAMERA = tf.compose_matrix(
    translate=[in2m(-6.73), 0, in2m(8.23)], angles=[0, np.deg2rad(-45), 0]
)
ROBOT_TO_CAMERA = ROBOT_TO_LAUNCHER_PIVOT.dot(LAUNCHER_PIVOT_TO_LAUNCHER).dot(
    LAUNCHER_TO_CAMERA
)

FIELD_LENGTH = in2m(12 * 54)  # m
FIELD_WIDTH = in2m(12 * 27)  # m
FIELD_HALF_LENGTH = FIELD_LENGTH * 0.5
FIELD_HALF_WIDTH = FIELD_WIDTH * 0.5

TARGET_CENTER_X = 0.0  # m
TARGET_CENTER_Y = 0.0  # m
TARGET_CENTER_Z = 2.60  # m
TARGET_DIAMETER = 1.36  # m
TARGET_RADIUS = 0.5 * TARGET_DIAMETER  # m
# TARGET_CENTER_X = in2m(0)  # Book depository
# TARGET_CENTER_Y = in2m(0)
# TARGET_CENTER_Z = in2m(91)

# NUM_LANDMARKS = 16
# LANDMARK_ANGULAR_SPACING = 2.0 * np.pi / NUM_LANDMARKS
# LANDMARK_GAP_ANGULAR_OFFSET = np.deg2rad(24)
# LANDMARKS = []

# # Create landmarks CCW, with idx=0 being the first landmark from the +X axis.
# landmark_start_angle = LANDMARK_GAP_ANGULAR_OFFSET - 0.5 * LANDMARK_ANGULAR_SPACING
# for idx in range(NUM_LANDMARKS):
#     theta = landmark_start_angle * idx * LANDMARK_ANGULAR_SPACING
#     dx = GOAL_RADIUS * np.cos(theta)
#     dy = GOAL_RADIUS * np.sin(theta)
#     LANDMARKS.append(Landmark(
#         idx,
#         TARGET_CENTER_X + dx,
#         TARGET_CENTER_Y + dy,
#         TARGET_CENTER_Z,
#     ))

PARTICLE_LIKELIHOOD_ACCEPT_TOLERANCE = 1e-3
FRACTION_OF_PARTICLES_OVER_TOLERANCE = 0.01
REINITIALIZE_THRESHOLD = 10
INIT_ALLOWANCE = 30

# Helper function to compute point on target circle closest to the given x, y
def get_point_on_target_circle(x, y):
    norm = np.sqrt(x * x + y * y)
    scalar = TARGET_RADIUS / norm
    dx = x * scalar
    dy = y * scalar
    zs = np.full(dx.shape, TARGET_CENTER_Z)

    points = np.empty((x.shape[0] if x.shape else 1, 3, 1))
    points[:, 0, 0] = np.array([TARGET_CENTER_X + dx])
    points[:, 1, 0] = np.array([TARGET_CENTER_Y + dy])
    points[:, 2, 0] = zs
    return points


class ParticleFilter:
    def __init__(self, num_particles: int, priori: Odometry, targets):
        # Always start with same random seed for reproducibility
        np.random.seed(0)

        self.num_particles = num_particles
        # Keep track of last odometry to compute deltas.
        self.last_odom = priori
        self.targets = targets
        self.initialize()

    def initialize(self, reinitialize=False):
        # Represent particles as an Nx4x4 matrix
        # N is the number of particles and each 4x4 matrix is a 3D transformation matrix
        self.particles = np.repeat(
            np.eye(4)[np.newaxis, :, :], self.num_particles, axis=0
        )
        # We need to keep track of the non-wrapping continuous yaw separately, since the 3D rotation matrix loses this info.
        self.particle_continuous_yaw = np.empty((self.num_particles, 1))
        self.particle_weights = np.full(
            self.num_particles, 1.0 / float(self.num_particles)
        )
        self.particle_indexes = np.array(
            range(self.num_particles)
        )  # Used for resampling

        # Priori sigmas
        sigmaX = 3.0 if reinitialize else 5e-2
        sigmaY = 3.0 if reinitialize else 5e-2
        sigmaTheta = 1e-3  # We trust our yaw
        xs = np.random.normal(self.last_odom.x, sigmaX, self.num_particles)
        ys = np.random.normal(self.last_odom.y, sigmaY, self.num_particles)
        self.particle_continuous_yaw = np.random.normal(
            self.last_odom.theta, sigmaTheta, self.num_particles
        )
        set_x(self.particles, xs)
        set_y(self.particles, ys)
        set_yaw(self.particles, self.particle_continuous_yaw)

        # Used to reinitialize
        self.consecutive_bad_measurements = 0
        self.n_since_init = 0

    def predict(self, odometry: Odometry):
        # Compute delta motion
        T = tf.compose_matrix(
            translate=[odometry.x, odometry.y, 0], angles=[0, 0, odometry.theta]
        )
        last_T = tf.compose_matrix(
            translate=[self.last_odom.x, self.last_odom.y, 0],
            angles=[0, 0, self.last_odom.theta],
        )
        self.last_odom = odometry

        M = np.linalg.inv(last_T).dot(T)
        _, _, angles, trans, _ = tf.decompose_matrix(M)
        dx, dy, _ = trans
        _, _, dtheta = angles

        # Precompute gaussian noise to apply to x, y, and theta for each particle.
        # Add translational noise in polar coordinates.
        dr, dphi = cart2pol(dx, dy)
        rSigma = (
            6e-1 * dr + 1e-2
        )  # Scale uncertianty by distance + a constant uncertainty
        phiSigma = 1e-1
        rNoise = np.random.normal(0.0, rSigma, size=self.num_particles)
        phiNoise = np.random.normal(0.0, phiSigma, size=self.num_particles)
        noisyX, noisyY = pol2cart(dr + rNoise, dphi + phiNoise)

        # Rotational noise
        thetaSigma = 1e-6
        thetaNoise = np.random.normal(0.0, thetaSigma, size=self.num_particles)

        # Transform each particle by value + noise.
        transforms = np.repeat(np.eye(4)[np.newaxis, :, :], self.num_particles, axis=0)
        set_x(transforms, noisyX)
        set_y(transforms, noisyY)
        dTheta = dtheta + thetaNoise
        set_yaw(transforms, dTheta)

        self.particles = np.matmul(self.particles, transforms)
        self.particle_continuous_yaw += dTheta

    def update(self, measurements: List[Vision]):
        boundary_update = self.update_field_boundaries()
        vision_update = self.update_vision(measurements)
        if boundary_update or vision_update:
            self.resample()
        return vision_update

    def update_field_boundaries(self):
        xs = (get_x(self.particles),)
        ys = (get_y(self.particles),)

        within_positive_x = np.less(xs, FIELD_HALF_LENGTH - MIN_ROBOT_DIMENSION)
        within_negative_x = np.greater(xs, -FIELD_HALF_LENGTH + MIN_ROBOT_DIMENSION)
        within_positive_y = np.less(ys, FIELD_HALF_WIDTH - MIN_ROBOT_DIMENSION)
        within_negative_y = np.greater(ys, -FIELD_HALF_WIDTH + MIN_ROBOT_DIMENSION)

        in_bounds_x = np.logical_and(within_positive_x, within_negative_x)
        in_bounds_y = np.logical_and(within_positive_y, within_negative_y)
        in_bounds = np.logical_and(in_bounds_x, in_bounds_y)[0]

        if not np.all(in_bounds):
            # Update particle weights
            self.particle_weights *= in_bounds.astype(float)
            self.normalize_weights()
            return True
        return False

    def update_vision(self, measurements: List[Vision]):
        if measurements is None or len(measurements) < 1:
            return False

        # Compute and combine likelihoods for each target
        likelihoods = np.ones((self.num_particles, ))
        for measurement in measurements:
            likelihoods *= self.compute_single_measurement_likelihoods(measurement)

        self.n_since_init += 1

        # Check if |FRACTION_OF_PARTICLES_OVER_TOLERANCE| have a likelihood greater than |PARTICLE_LIKELIHOOD_ACCEPT_TOLERANCE|
        over_tolerance = np.greater(likelihoods, PARTICLE_LIKELIHOOD_ACCEPT_TOLERANCE)
        if (
            np.sum(over_tolerance)
            > FRACTION_OF_PARTICLES_OVER_TOLERANCE * self.num_particles
        ):
            self.consecutive_bad_measurements = 0
            # Update particle weights
            self.particle_weights *= likelihoods
            self.normalize_weights()
            return True
        else:
            self.consecutive_bad_measurements += 1
            print(
                f"Bad measurement {self.consecutive_bad_measurements}/{REINITIALIZE_THRESHOLD}"
            )

        if (
            self.consecutive_bad_measurements > REINITIALIZE_THRESHOLD
            and self.n_since_init > INIT_ALLOWANCE
        ):
            print("REINITIALIZING!!!!!!!!!!!!!!!!!!")
            self.initialize(reinitialize=True)

        return False

    def compute_single_measurement_likelihoods(self, measurement):
        target = self.targets.get(measurement.targetID, None)
        camera_T = np.matmul(self.particles, ROBOT_TO_CAMERA)
        if target is None:
            # Compute the closest point of the target in the camera frame.
            target_point = get_point_on_target_circle(
                camera_T[:, 0, 3], camera_T[:, 1, 3]
            )
        else:
            # Compute the point of the target in the camera frame.
            target_point = np.empty((1, 3, 1))
            target_point[:, 0, 0] = target[0]
            target_point[:, 1, 0] = target[1]
            target_point[:, 2, 0] = target[2]
        
        # Compare the expected measurement with the actual measurement
        point = get_point_in_frame(camera_T, target_point)
        expected_measurement = cart2besph(point[:, 0], point[:, 1], point[:, 2])
        delta_measurement = expected_measurement - np.array(
            [measurement.bearing, measurement.elevation]
        )
        # TODO: Figure out a cleaner way to do this
        delta_measurement_stacked = np.expand_dims(delta_measurement, axis=2)
        delta_measurement_stacked_transpose = np.expand_dims(delta_measurement, axis=1)

        # Compute the likelihood of the measurement
        beSigma = 5e-3
        elSigma = 5e-3
        covariance_matrix = np.diag(np.array([beSigma, elSigma]))
        repeated_inv_covariance = np.repeat(
            np.linalg.inv(covariance_matrix)[np.newaxis, :, :],
            self.num_particles,
            axis=0,
        )
        scalar_term = 1.0 / np.sqrt(
            ((2 * np.pi) ** 2) * np.linalg.det(covariance_matrix)
        )
        main_term = -0.5 * np.matmul(
            np.matmul(delta_measurement_stacked_transpose, repeated_inv_covariance),
            delta_measurement_stacked,
        )
        likelihoods = np.ndarray.flatten(scalar_term * np.exp(main_term))
        return likelihoods

    def normalize_weights(self):
        weight_sum = sum(self.particle_weights)
        if weight_sum == 0:
            self.initialize(reinitialize=True)
        else:
            self.particle_weights /= weight_sum

    def resample(self):
        chosen_indices = np.random.choice(
            self.particle_indexes, size=self.num_particles, p=self.particle_weights
        )
        self.particles = self.particles[chosen_indices, :, :]
        self.particle_continuous_yaw = self.particle_continuous_yaw[chosen_indices]
        self.particle_weights = np.full(
            self.num_particles, 1.0 / float(self.num_particles)
        )

    def get_best_estimate(self):
        weighted_x = np.sum(get_x(self.particles) * self.particle_weights)
        weighted_y = np.sum(get_y(self.particles) * self.particle_weights)
        weighted_yaw = np.sum(self.particle_continuous_yaw * self.particle_weights)
        return weighted_x, weighted_y, weighted_yaw

    def plot_particles(self, plt, has_vision=False):
        plt.scatter(
            get_x(self.particles),
            get_y(self.particles),
            s=(self.particle_weights * 100),
            marker=".",
        )
        weighted_x = np.sum(get_x(self.particles) * self.particle_weights)
        weighted_y = np.sum(get_y(self.particles) * self.particle_weights)
        plt.scatter(
            weighted_x, weighted_y, marker="o", color="C2" if has_vision else "C3"
        )
        plt.plot()

    def plot_estimate(self, plt, has_vision=False, facing_our_goal=True):
        x, y, _ = self.get_best_estimate()
        plt.scatter(
            x,
            y,
            s=5,
            marker="o",
            color=("g" if facing_our_goal else "b") if has_vision else "r",
        )
        plt.plot()
