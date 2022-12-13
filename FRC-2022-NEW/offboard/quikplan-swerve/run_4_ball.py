import matplotlib.pyplot as plt
import numpy as np

from utils.field import Field
from utils.robot import Robot
from utils.quikplan import QuikPlan, StoppedPoseConstraint, PoseConstraint, StoppedXYConstraint, \
    XYConstraint, AngularConstraint, XConstraint, YConstraint, \
    GoalConstraint, SpeedConstraint, StayStoppedConstraint
from utils.helpers import write_to_csv

SCORING_DIST = 2.5

# Create the field
field = Field()

# Create the robot model
robot = Robot()

# Configure the optimizer
start_pose = (-1.45, -1.7, np.deg2rad(45))
qp = QuikPlan(
    field,
    robot,
    start_pose,
    [StoppedPoseConstraint(start_pose)],
    15.0
)

# Pick up ball
waypoint3 = (-2.3, -2.25, np.pi)
qp.add_waypoint(
    waypoint3,
    10,
    end_constraints=[PoseConstraint(waypoint3), SpeedConstraint(0.5)]
)
waypoint4 = (-3.0, -2.25, np.pi)
qp.add_waypoint(
    waypoint4,
    10,
    end_constraints=[StoppedPoseConstraint(waypoint4)]
)

# Move to scoring position
waypoint5 = (-2.5, -2.5, 0.25 * np.pi)
qp.add_waypoint(
    waypoint5,
    10,
    end_constraints=[XYConstraint(waypoint5)]
)
qp.add_waypoint(
    waypoint5,
    10,
    end_constraints=[GoalConstraint(SCORING_DIST), SpeedConstraint(0.0)]
)

qp.add_waypoint(
    waypoint5,
    10,
    intermediate_constraints=[StayStoppedConstraint(1.5)]
)

# Drive to feeder station
waypoint6 = (-6.4, -2.5, -np.pi * 0.75)
qp.add_waypoint(
    waypoint6,
    10,
    end_constraints=[PoseConstraint(waypoint6), SpeedConstraint(0.5)]
)
waypoint7 = (-6.8, -2.8, -np.pi * 0.75)
qp.add_waypoint(
    waypoint7,
    10,
    end_constraints=[StoppedPoseConstraint(waypoint7)]
)

waypoint7b = (-6.4, -2.5, -np.pi * 0.75)
qp.add_waypoint(
    waypoint7b,
    10,
    end_constraints=[StoppedPoseConstraint(waypoint7b)]
)

qp.add_waypoint(
    waypoint7b,
    10,
    intermediate_constraints=[StayStoppedConstraint(1.5)]
)

# Move to scoring position
waypoint8 = (-2.5, -2.5, -np.pi * 2.25)
qp.add_waypoint(
    waypoint8,
    10,
    end_constraints=[XYConstraint(waypoint8)]
)
qp.add_waypoint(
    waypoint8,
    10,
    end_constraints=[GoalConstraint(SCORING_DIST), SpeedConstraint(0.0)]
)

qp.add_waypoint(
    waypoint8,
    10,
    intermediate_constraints=[StayStoppedConstraint(1.5)]
)

# Plan the trajectory
# qp.plot_init()
traj = qp.plan()
write_to_csv(traj, '04_ball')

# Plot
field.plot_traj(robot, traj, '04_ball.png', save=True)

# Animate
field.anim_traj(robot, traj, '04_ball.gif', save_gif=False)
