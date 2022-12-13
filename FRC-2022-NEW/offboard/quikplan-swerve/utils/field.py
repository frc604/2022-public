import matplotlib.animation as animation
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import os

from utils.helpers import in2m

# (0, 0) at the center of the field
# +x points toward the opposing alliance station wall
# +y points left
class Field(object):
    FIELD_IMAGE = os.path.join(os.path.dirname(__file__), '../../../field.png')
    LENGTH = 15.98  # m
    WIDTH = 8.21  # m

    OBSTACLES = []  # (x, y, radius)

    BALLS = [
        (-0.65, -3.83),
        (-3.09, -2.24),
        (-3.19, 2.08),
        (-6.98, -2.98),  # by loading station
    ]
    BALL_RADIUS = in2m(3.5)

    GOAL = (0.0, 0.0)
    YEET_POINT = (-7.0, 2.6)

    def plot_field(self, ax):
        img = mpimg.imread(self.FIELD_IMAGE)
        ax.imshow(img, extent=[-self.LENGTH * 0.5, self.LENGTH * 0.5, -self.WIDTH * 0.5, self.WIDTH * 0.5])
        # Plot obstacles
        for x, y, r in self.OBSTACLES:
            ax.add_artist(plt.Circle((x, y), r, color='m'))
        # Plot balls
        for x, y in self.BALLS:
            ax.add_artist(plt.Circle((x, y), self.BALL_RADIUS, color='r'))

    def plot_traj(self, robot, traj, save_file, plot_mod=10, save=False):
        fig, ax = plt.subplots()
        
        # Plot field and path
        self.plot_field(ax)
        plt.scatter(traj[:, 1], traj[:, 2], marker='.', color=['g' if shoot else ('b' if yeet else 'r') for (shoot, yeet) in traj[:, 7:9]])

        traj_size = traj.shape[0]
        for k in range(traj_size):
            if k % plot_mod == 0 or k == traj_size - 1:
                robot.plot(ax, traj[k, 1:4])

        fig.set_size_inches(18, 9)
        plt.savefig(os.path.join(os.path.dirname(__file__), "../plots/{}".format(save_file)))
        plt.show()

    def anim_traj(self, robot, traj, save_file, save_gif=False):
        fig, ax = plt.subplots()

        # Plot field and path
        self.plot_field(ax)
        plt.scatter(traj[:, 1], traj[:, 2], marker='.', color=['g' if shoot else ('b' if yeet else 'r') for (shoot, yeet) in traj[:, 7:9]])

        # Plot first pose
        robot.plot(ax, traj[0, 1:4])
        # Plot last pose
        robot.plot(ax, traj[-1, 1:4])

        # Animation function
        def animate(i):
            print("Rendering Frame: {}".format(i))
            # Hack to remove the old robot poses
            ax.collections = ax.collections[:7]
            robot.plot(ax, traj[i, 1:4])
            return ax.collections

        anim = animation.FuncAnimation(
            fig, animate, frames=traj.shape[0], interval=20, blit=True, repeat=True
        )
        if save_gif:
            anim.save("out.gif", writer="pillow", fps=50)
        plt.show()
