import matplotlib.pyplot as plt
import numpy as np

from utils.field import Field
from utils.robot import Robot
from utils.quikplan import QuikPlan, StoppedPoseConstraint, PoseConstraint, StoppedXYConstraint, \
    XYConstraint, AngularConstraint, XConstraint, YConstraint, \
    GoalConstraint, SpeedConstraint, StayStoppedConstraint, YeetConstraint
from utils.helpers import write_to_csv

SCORING_DIST = 2.5

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
    8.0,  # yeet_lockout
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
    intermediate_constraints=[StayStoppedConstraint(2.5)]
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
    end_constraints=[PoseConstraint(waypoint3), SpeedConstraint(0.5)]
)
waypoint4 = (-3.1, -2.2, -np.pi * 1.25)
qp.add_waypoint(
    waypoint4,
    10,
    end_constraints=[PoseConstraint(waypoint4)]
)

# Move to scoring position
waypoint5 = (-2.5, -2.5, -np.pi * 2.25)
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

# Pick up opponent's ball
waypoint6 = (0.6, -3.0, -np.pi * 2.5)
qp.add_waypoint(
    waypoint6,
    10,
    end_constraints=[PoseConstraint(waypoint6), SpeedConstraint(0.5)]
)
waypoint7 = (0.8, -3.5, -np.pi * 2.5)
qp.add_waypoint(
    waypoint7,
    10,
    end_constraints=[StoppedPoseConstraint(waypoint7)]
)

# Yeet ball
waypoint5 = (0.0, -3.0, -np.pi * 3.25)
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

# Get ready for match
waypoint6 = (0.0, -3.0, -np.pi * 4.0)
qp.add_waypoint(
    waypoint6,
    10,
    end_constraints=[StoppedPoseConstraint(waypoint6)]
)

# Plan the trajectory
# qp.plot_init()
traj = qp.plan()
write_to_csv(traj, '03_ball')

# Plot
field.plot_traj(robot, traj, '03_ball.png', save=True)

# Animate
field.anim_traj(robot, traj, '03_ball.gif', save_gif=False)
