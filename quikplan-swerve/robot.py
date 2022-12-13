import casadi as ca
import matplotlib as mpl
import numpy as np

from helpers import in2m, rotate_around_origin, transform_geometry

G = 9.81  # m/s/s
MODULE_X_DIST = in2m(10.25)  # m
MODULE_Y_DIST = in2m(10.25)  # m

class Robot(object):
    def __init__(self):
        # Geometry
        self.WIDTH = in2m(34)  # m
        self.LENGTH = in2m(34)  # m
        self.GEOMETRY = [
            [
                (self.LENGTH / 2, self.WIDTH / 2),
                (self.LENGTH / 2, -self.WIDTH / 2),
            ],  # Front
            [
                (-self.LENGTH / 2, self.WIDTH / 2),
                (-self.LENGTH / 2, -self.WIDTH / 2),
            ],  # Back
            [
                (self.LENGTH / 2, self.WIDTH / 2),
                (-self.LENGTH / 2, self.WIDTH / 2),
            ],  # Left
            [
                (self.LENGTH / 2, -self.WIDTH / 2),
                (-self.LENGTH / 2, -self.WIDTH / 2),
            ],  # Right
        ]
        self.OBSTACLE_BUFFER = 0.1  # m
        self.WHEEL_DIA = in2m(3.94)  # m
        self.MODULE_POSITIONS = [
            (MODULE_X_DIST, MODULE_Y_DIST),  # ID 0: FL
            (MODULE_X_DIST, -MODULE_Y_DIST),  # ID 1: FR
            (-MODULE_X_DIST, -MODULE_Y_DIST),  # ID 2: BR
            (-MODULE_X_DIST, MODULE_Y_DIST),  # ID 3: BL
        ]
        self.NUM_MODULES = len(self.MODULE_POSITIONS)

        # Mass / Inertia
        self.MU = 0.8
        self.MASS = 60.0  # kg
        self.J = (
            self.MASS * (self.LENGTH ** 2 + self.WIDTH ** 2)
        ) / 12.0  # Moment of inertia
        self.J *= 1.1  # Conservative scaling

        # Motors
        self.MOTOR_MAX_TORQUE  = 0.8  # N*m @ 40A
        self.MOTOR_MAX_RPM  = 4000  # RPM @ 10V
        self.DRIVE_RATIO = 6.75  # 6.75:1
        self.WHEEL_40A_FORCE = (
            self.MOTOR_MAX_TORQUE
            * self.DRIVE_RATIO
            / (self.WHEEL_DIA / 2.0)
        )

    def plot(self, ax, state, robot_axis_plot_size=0.1):
        pose = state[:3]

        # Plot robot geometry
        ax.add_collection(
            mpl.collections.LineCollection(
                transform_geometry(self.GEOMETRY, pose), color="k"
            )
        )
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
    
    def dynamics_model(self, x, u):
        # dx/dt = f(x, u)
        return ca.vertcat(
            x[3],
            x[4],
            x[5],
            x[6],
            x[7],
            x[8],
            u[0],
            u[1],
            u[2],
        )

    def get_vector_to_module(self, module_idx, theta):
        module_x = self.MODULE_POSITIONS[module_idx][0]
        module_y = self.MODULE_POSITIONS[module_idx][1]
        rho = np.sqrt(module_x * module_x + module_y * module_y)
        phi = np.arctan2(module_y, module_x)
        return rho * np.cos(theta + phi), rho * np.sin(theta + phi)

    def apply_speed_constraint(self, opti, x, u, i, speed_limit):
        # Impose additional speed limit on top of motor speed limit
        theta = x[2, i]
        xDot = x[3, i]
        yDot = x[4, i]
        thetaDot = x[5, i]

        r0x, r0y = self.get_vector_to_module(0, theta)
        r1x, r1y = self.get_vector_to_module(1, theta)
        r2x, r2y = self.get_vector_to_module(2, theta)
        r3x, r3y = self.get_vector_to_module(3, theta)

        v0x = xDot - r0y * thetaDot
        v0y = yDot + r0x * thetaDot
        v1x = xDot - r1y * thetaDot
        v1y = yDot + r1x * thetaDot
        v2x = xDot - r2y * thetaDot
        v2y = yDot + r2x * thetaDot
        v3x = xDot - r3y * thetaDot
        v3y = yDot + r3x * thetaDot

        v0_sq = v0x * v0x + v0y * v0y
        v1_sq = v1x * v1x + v1y * v1y
        v2_sq = v2x * v2x + v2y * v2y
        v3_sq = v3x * v3x + v3y * v3y

        max_v_sq = speed_limit * speed_limit
        opti.subject_to(opti.bounded(-max_v_sq, v0_sq, max_v_sq))
        opti.subject_to(opti.bounded(-max_v_sq, v1_sq, max_v_sq))
        opti.subject_to(opti.bounded(-max_v_sq, v2_sq, max_v_sq))
        opti.subject_to(opti.bounded(-max_v_sq, v3_sq, max_v_sq))

    def apply_module_constraints(self, opti, x, u, N):
        theta = x[2, :]
        xDot = x[3, :]
        yDot = x[4, :]
        thetaDot = x[5, :]
        xDotDot = x[6, :]
        yDotDot = x[7, :]
        thetaDotDot = x[8, :]

        # Store velocities and forces at each module for plotting.
        v0s = []  # Velocites
        v1s = []
        v2s = []
        v3s = []
        f0s = []  # Wheel-aligned forces
        f1s = []
        f2s = []
        f3s = []

        # For each state, compute module vectors and apply constraints
        for k in range(N + 1):
            r0x, r0y = self.get_vector_to_module(0, theta[k])
            r1x, r1y = self.get_vector_to_module(1, theta[k])
            r2x, r2y = self.get_vector_to_module(2, theta[k])
            r3x, r3y = self.get_vector_to_module(3, theta[k])

            v0x = xDot[k] - r0y * thetaDot[k]
            v0y = yDot[k] + r0x * thetaDot[k]
            v1x = xDot[k] - r1y * thetaDot[k]
            v1y = yDot[k] + r1x * thetaDot[k]
            v2x = xDot[k] - r2y * thetaDot[k]
            v2y = yDot[k] + r2x * thetaDot[k]
            v3x = xDot[k] - r3y * thetaDot[k]
            v3y = yDot[k] + r3x * thetaDot[k]

            v0_sq = v0x * v0x + v0y * v0y
            v1_sq = v1x * v1x + v1y * v1y
            v2_sq = v2x * v2x + v2y * v2y
            v3_sq = v3x * v3x + v3y * v3y

            max_v = (
                (self.MOTOR_MAX_RPM / 60 / self.DRIVE_RATIO) * np.pi * self.WHEEL_DIA
            )
            max_v_sq = max_v * max_v
            opti.subject_to(opti.bounded(-max_v_sq, v0_sq, max_v_sq))
            opti.subject_to(opti.bounded(-max_v_sq, v1_sq, max_v_sq))
            opti.subject_to(opti.bounded(-max_v_sq, v2_sq, max_v_sq))
            opti.subject_to(opti.bounded(-max_v_sq, v3_sq, max_v_sq))

            # a0x = xDotDot[k] - r0y * thetaDotDot[k]
            # a0y = yDotDot[k] + r0x * thetaDotDot[k]
            # a1x = xDotDot[k] - r1y * thetaDotDot[k]
            # a1y = yDotDot[k] + r1x * thetaDotDot[k]
            # a2x = xDotDot[k] - r2y * thetaDotDot[k]
            # a2y = yDotDot[k] + r2x * thetaDotDot[k]
            # a3x = xDotDot[k] - r3y * thetaDotDot[k]
            # a3y = yDotDot[k] + r3x * thetaDotDot[k]

            # Axis-aligned forces needed to achieve this linear acceleration.
            Fx = self.MASS * xDotDot[k]
            Fy = self.MASS * yDotDot[k]
            # Torque about the center of rotation needed to achieve this angular acceleration.
            T = self.J * thetaDotDot[k]

            # Define force variables for each module.
            f0x = opti.variable()
            f0y = opti.variable()
            f1x = opti.variable()
            f1y = opti.variable()
            f2x = opti.variable()
            f2y = opti.variable()
            f3x = opti.variable()
            f3y = opti.variable()

            # Forces must respect motor and friction limits.
            max_friction_force = self.MU * self.MASS * G / self.NUM_MODULES
            force_limit = min(self.WHEEL_40A_FORCE, max_friction_force)
            opti.subject_to(
                opti.bounded(0, f0x * f0x + f0y * f0y, force_limit * force_limit)
            )
            opti.subject_to(
                opti.bounded(0, f1x * f1x + f1y * f1y, force_limit * force_limit)
            )
            opti.subject_to(
                opti.bounded(0, f2x * f2x + f2y * f2y, force_limit * force_limit)
            )
            opti.subject_to(
                opti.bounded(0, f3x * f3x + f3y * f3y, force_limit * force_limit)
            )

            # Sum of all linear force components must equal |Fx| and |Fy|.
            opti.subject_to(Fx == f0x + f1x + f2x + f3x)
            opti.subject_to(Fy == f0y + f1y + f2y + f3y)

            # Sum of all torques must equal |T|.
            r0 = np.array([r0x, r0y])
            f0 = np.array([f0x, f0y])
            m0_dir = np.array([-r0y, r0x])
            f0t = np.dot(f0, m0_dir) / np.linalg.norm(m0_dir)

            r1 = np.array([r1x, r1y])
            f1 = np.array([f1x, f1y])
            m1_dir = np.array([-r1y, r1x])
            f1t = np.dot(f1, m1_dir) / np.linalg.norm(m1_dir)

            r2 = np.array([r2x, r2y])
            f2 = np.array([f2x, f2y])
            m2_dir = np.array([-r2y, r2x])
            f2t = np.dot(f2, m2_dir) / np.linalg.norm(m2_dir)

            r3 = np.array([r3x, r3y])
            f3 = np.array([f3x, f3y])
            m3_dir = np.array([-r3y, r3x])
            f3t = np.dot(f3, m3_dir) / np.linalg.norm(m3_dir)

            opti.subject_to(
                T
                == f0t * np.linalg.norm(r0)
                + f1t * np.linalg.norm(r1)
                + f2t * np.linalg.norm(r2)
                + f3t * np.linalg.norm(r3)
            )

            # Store for plotting
            v0s.append(np.sqrt(v0_sq))
            v1s.append(np.sqrt(v1_sq))
            v2s.append(np.sqrt(v2_sq))
            v3s.append(np.sqrt(v3_sq))
            f0s.append(f0t)
            f1s.append(f1t)
            f2s.append(f2t)
            f3s.append(f3t)

        return (
            v0s,
            v1s,
            v2s,
            v3s,
            f0s,
            f1s,
            f2s,
            f3s,
        )

    def apply_obstacle_constraints(self, opti, xpos, ypos, theta, obstacles):
        for obx, oby, obr in obstacles:
            for p1, p2 in self.GEOMETRY:
                # Transform robot geometry to pose
                x1, y1 = rotate_around_origin(p1, theta)
                x2, y2 = rotate_around_origin(p2, theta)
                x1 += xpos
                y1 += ypos
                x2 += xpos
                y2 += ypos

                # Compute the closest distance between a point and a line segment
                px = x2 - x1
                py = y2 - y1
                norm = px * px + py * py
                u = ((obx - x1) * px + (oby - y1) * py) / norm
                u = ca.fmax(ca.fmin(u, 1), 0)
                x = x1 + u * px
                y = y1 + u * py
                dx = x - obx
                dy = y - oby

                dist = np.sqrt((dx * dx + dy * dy))
                opti.subject_to(dist > obr + self.OBSTACLE_BUFFER)
