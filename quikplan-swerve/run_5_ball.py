import matplotlib.pyplot as plt
import numpy as np

from field import Field
from robot import Robot
from quikplan import QuikPlan, StoppedPoseConstraint, PoseConstraint, StoppedXYConstraint, \
    XYConstraint, AngularConstraint, XConstraint, YConstraint, \
    GoalConstraint, SpeedConstraint, StayStoppedConstraint
from helpers import write_to_csv

SCORING_DIST = 2.5
FAR_SCORING_DIST = 3.0

# Create the field
field = Field()

# Create the robot model
robot = Robot()

# Configure the optimizer
start_pose = (-0.65, -2.3, np.deg2rad(91.5))  # Rear bumper points at the far ball
qp = QuikPlan(
    field,
    robot,
    start_pose,
    [StoppedPoseConstraint(start_pose)],
    15.0,  # yeet_lockout
)

# Score starting ball
waypoint0 = (-0.65, -2.3, np.pi * 0.5)
qp.add_waypoint(
    waypoint0,
    10,
    end_constraints=[GoalConstraint(SCORING_DIST), SpeedConstraint(0.0)]
)

qp.add_waypoint(
    waypoint0,
    10,
    intermediate_constraints=[StayStoppedConstraint(2.0)]
)

# Pick up first ball
waypoint1 = (-0.65, -3.0, -np.pi * 0.5)
qp.add_waypoint(
    waypoint1,
    10,
    end_constraints=[PoseConstraint(waypoint1), SpeedConstraint(0.5)]
)
waypoint2 = (-0.65, -3.5, -np.pi * 0.5)
qp.add_waypoint(
    waypoint2,
    10,
    end_constraints=[StoppedPoseConstraint(waypoint2)]
)

# Pick up second ball
waypoint3 = (-2.6, -2.6, -np.pi * 1.25)
qp.add_waypoint(
    waypoint3,
    10,
    end_constraints=[PoseConstraint(waypoint3)  ]
)
waypoint4 = (-3.1, -2.2, -np.pi * 1.25)
qp.add_waypoint(
    waypoint4,
    10,
    intermediate_constraints=[SpeedConstraint(1.0)],
    end_constraints=[PoseConstraint(waypoint4)]
)

# Move to scoring position
waypoint5 = (-2.6, -1.8, -np.pi * 2.25)
qp.add_waypoint(
    waypoint5,
    10,
    end_constraints=[XYConstraint(waypoint5)]
)
qp.add_waypoint(
    waypoint5,
    10,
    end_constraints=[GoalConstraint(FAR_SCORING_DIST), SpeedConstraint(0.0)]
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
waypoint8 = (-2.6, -1.8, -np.pi * 2.25)
qp.add_waypoint(
    waypoint8,
    10,
    end_constraints=[XYConstraint(waypoint8)]
)
qp.add_waypoint(
    waypoint8,
    10,
    end_constraints=[GoalConstraint(FAR_SCORING_DIST), SpeedConstraint(0.0)]
)

qp.add_waypoint(
    waypoint8,
    10,
    intermediate_constraints=[StayStoppedConstraint(1.5)]
)

# Plan the trajectory
# qp.plot_init()
traj = qp.plan()
write_to_csv(traj, './output/05_ball.csv')

# Plot
field.plot_traj(robot, traj, './output/05_ball.png', save=True)

# Animate
field.anim_traj(robot, traj, '05_ball.gif', save_gif=False)
