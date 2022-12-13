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
start_pose = (-2.2, 0.0, 0.0)
qp = QuikPlan(
    field,
    robot,
    start_pose,
    [StoppedPoseConstraint(start_pose)],
)

# Score starting ball
qp.add_waypoint(
    start_pose,
    10,
    intermediate_constraints=[StayStoppedConstraint(2.5)]
)

# Back up
waypoint2 = (-4.0, 0.0, 0.0)
qp.add_waypoint(
    waypoint2,
    10,
    end_constraints=[StoppedPoseConstraint(waypoint2)]
)

# Plan the trajectory
# qp.plot_init()
traj = qp.plan()
write_to_csv(traj, '01_ball')

# Plot
field.plot_traj(robot, traj, '01_ball.png', save=True)

# Animate
field.anim_traj(robot, traj, '01_ball.gif', save_gif=False)
