import matplotlib.pyplot as plt
import numpy as np

from field import Field
from robot import Robot
from quikplan import QuikPlan, StoppedPoseConstraint, PoseConstraint, StoppedXYConstraint, \
    XYConstraint, AngularConstraint, XConstraint, YConstraint, \
    GoalConstraint, SpeedConstraint, StayStoppedConstraint, YeetConstraint
from helpers import write_to_csv

SCORING_DIST = 2.5

# Create the field
field = Field()

# Create the robot model
robot = Robot()

# Configure the optimizer
start_pose = (-1.95, 1.05, -np.pi * 0.25)
qp = QuikPlan(
    field,
    robot,
    start_pose,
    [StoppedPoseConstraint(start_pose)],
)

# Score starting ball
goal_guess = (-2.35, 1.45, -np.pi * 0.25)
qp.add_waypoint(
    goal_guess,
    10,
    end_constraints=[GoalConstraint(SCORING_DIST), SpeedConstraint(0.0)]
)

qp.add_waypoint(
    goal_guess,
    10,
    intermediate_constraints=[StayStoppedConstraint(2.5)]
)

# Pick up our ball
waypoint1 = (-2.50, 1.55, np.pi * 0.75)
qp.add_waypoint(
    waypoint1,
    10,
    end_constraints=[PoseConstraint(waypoint1), SpeedConstraint(0.5)]
)
waypoint2 = (-3.0, 1.95, np.pi * 0.75)
qp.add_waypoint(
    waypoint2,
    10,
    end_constraints=[StoppedPoseConstraint(waypoint2)]
)

# Move to scoring position
qp.add_waypoint(
    goal_guess,
    10,
    end_constraints=[GoalConstraint(SCORING_DIST), SpeedConstraint(0.0)]
)
qp.add_waypoint(
    goal_guess,
    10,
    intermediate_constraints=[StayStoppedConstraint(1.5)]
)

# Pick up opponent's ball
waypoint3 = (-2.2, 2.5, np.pi * 0.5)
qp.add_waypoint(
    waypoint3,
    10,
    end_constraints=[PoseConstraint(waypoint3), SpeedConstraint(0.5)]
)
waypoint4 = (-2.2, 3.0, np.pi * 0.5)
qp.add_waypoint(
    waypoint4,
    10,
    end_constraints=[StoppedPoseConstraint(waypoint4)]
)

# Yeet first ball
waypoint5 = (-3.7, 2.5, np.pi * 0.75)
qp.add_waypoint(
    waypoint5,
    10,
    end_constraints=[StoppedXYConstraint(waypoint5), YeetConstraint()]
)
qp.add_waypoint(
    waypoint5,
    10,
    intermediate_constraints=[StayStoppedConstraint(0.5)]
)

# Pick up opponent's ball 2
waypoint6 = (-3.7, -0.1, np.pi * 1.5)
qp.add_waypoint(
    waypoint6,
    10,
    end_constraints=[PoseConstraint(waypoint6), SpeedConstraint(0.5)]
)
waypoint7 = (-3.7, -0.6, np.pi * 1.5)
qp.add_waypoint(
    waypoint7,
    10,
    end_constraints=[StoppedPoseConstraint(waypoint7)]
)

# Yeet second ball
waypoint8 = (-3.7, 2.0, np.pi * 0.75)
qp.add_waypoint(
    waypoint8,
    10,
    end_constraints=[StoppedXYConstraint(waypoint8), YeetConstraint()]
)
qp.add_waypoint(
    waypoint8,
    10,
    intermediate_constraints=[StayStoppedConstraint(0.5)]
)

# Get ready for match
waypoint9 = (-3.0, 3.0, 0)
qp.add_waypoint(
    waypoint9,
    10,
    end_constraints=[StoppedPoseConstraint(waypoint9)]
)

# Plan the trajectory
# qp.plot_init()
traj = qp.plan()
write_to_csv(traj, './output/02_ball_left.csv')

# Plot
field.plot_traj(robot, traj, './output/02_ball_left.png', save=True)

# Animate
field.anim_traj(robot, traj, '02_ball_left.gif', save_gif=False)
