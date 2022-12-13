import matplotlib.pyplot as plt
import numpy as np

from field import Field
from robot import Robot
from quikplan import QuikPlan, StoppedPoseConstraint
from helpers import write_to_csv

# Create the field
field = Field()

# Create the robot model
robot = Robot()

# Configure the optimizer
start_pose = (0, 0, 0)
qp = QuikPlan(
    field,
    robot,
    start_pose,
    [StoppedPoseConstraint(start_pose)]
)
waypoint2 = (1.0, 0.0, np.pi)
qp.add_waypoint(
    waypoint2,
    50,
    end_constraints=[StoppedPoseConstraint(waypoint2)]
)

# Plan the trajectory
traj = qp.plan()
write_to_csv(traj, './output/simple_test.csv')

# Plot
field.plot_traj(robot, traj, './output/simple_test.png', save=True)
plt.show()

# Animate
field.anim_traj(robot, traj, 'simple_test.gif', save_gif=False)
