import csv
import numpy as np
import matplotlib.animation as animation
import matplotlib.pyplot as plt

from utils.field import Field
from utils.robot import Robot

def load_traj(filename):
    traj = []
    with open(filename) as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            traj.append([float(r) for r in row])
    return np.array(traj)

def animate_trajs(trajs):
    field = Field()
    robot = Robot()
    fig, ax = plt.subplots()

    for traj in trajs:
        # Plot field and path
        field.plot_field(ax)
        plt.scatter(traj[:, 1], traj[:, 2], marker='.')

    # Animation function
    def animate(i):
        print("Rendering Frame: {}".format(i))
        # Hack to remove the old robot poses
        ax.collections = ax.collections[:3]
        for traj in trajs:
            if i < traj.shape[0]:
                robot.plot(ax, traj[i, 1:4])
            else:
                robot.plot(ax, traj[-1, 1:4])
        return ax.collections

    max_traj_length = max([traj.shape[0] for traj in trajs]) + 10
    anim = animation.FuncAnimation(
        fig, animate, frames=max_traj_length, interval=20, blit=True, repeat=True
    )
    anim.save("out.gif", writer="pillow", fps=50)
    plt.show()

traj1 = load_traj('./output/05_ball.csv')
traj2 = load_traj('./output/06_ball.csv')
traj3 = load_traj('./output/08_ball.csv')
animate_trajs([traj1, traj2, traj3])
