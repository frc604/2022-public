from turtle import title
import matplotlib.pyplot as plt
from multiprocessing import Process, Manager, Queue
import numpy as np
from pyqtgraph.Qt import QtCore, QtGui
import transformations as tf
import pyqtgraph as pg

from data_utils import *
from helpers import in2m
from particle_filter import ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH, ROBOT_TO_CAMERA

FIELD_LENGTH = 15.98  # m
FIELD_WIDTH = 8.21  # m


ROBOT_COORDS = np.array(
    [
        [
            ROBOT_HALF_LENGTH,
            -ROBOT_HALF_LENGTH,
            -ROBOT_HALF_LENGTH,
            ROBOT_HALF_LENGTH,
            ROBOT_HALF_LENGTH,
        ],
        [
            ROBOT_HALF_WIDTH,
            ROBOT_HALF_WIDTH,
            -ROBOT_HALF_WIDTH,
            -ROBOT_HALF_WIDTH,
            ROBOT_HALF_WIDTH,
        ],
        [1.0, 1.0, 1.0, 1.0, 1.0],
    ]
)

LL_TRANSLATION = tf.translation_from_matrix(ROBOT_TO_CAMERA)
LL_FOV_LENGTH = 3  # m
LL_FOV = np.deg2rad(59.6)  # degrees

LL_CORDS = np.array(
    [
        [
            LL_TRANSLATION[0],
            LL_TRANSLATION[0] + LL_FOV_LENGTH,
            LL_TRANSLATION[0] + LL_FOV_LENGTH,
            LL_TRANSLATION[0],
        ],
        [
            LL_TRANSLATION[1],
            LL_TRANSLATION[1] + LL_FOV_LENGTH * np.tan(LL_FOV / 2),
            LL_TRANSLATION[1] - LL_FOV_LENGTH * np.tan(LL_FOV / 2),
            LL_TRANSLATION[1],
        ],
        [1.0, 1.0, 1.0, 1.0],
    ]
)

LL_BEARING_COORDS = np.array([[LL_FOV_LENGTH, 0.0], [0.0, 0.0], [1.0, 1.0]])

LL_FOV_PEN = pg.mkPen(color=pg.mkColor(63, 191, 63, 90), width=2)
LL_FILL_BRUSH = pg.mkBrush(63, 191, 63, 50)
BEARING_PEN = pg.mkPen(color=pg.mkColor(63, 191, 63, 255), width=4)


class CenteredArrowItem(pg.ArrowItem):
    def paint(self, p, *args):
        p.translate(-self.boundingRect().center())
        pg.ArrowItem.paint(self, p, *args)


class Plotter:
    def __init__(self, num_particles, is_red=True):
        self.num_particles = num_particles
        self.is_red = is_red

    def run(self):
        self.win = pg.GraphicsLayoutWidget(title="QuixPF", show=True)
        self.win.resize(1000, 500)

        # Create 2D plotting area
        plot = self.win.addPlot()
        plot.setAspectLocked()

        # Show field
        img = pg.ImageItem()
        img.setImage(np.rot90(plt.imread("field.png"), k=3 if self.is_red else 1))
        img.setRect(-FIELD_LENGTH * 0.5, -FIELD_WIDTH * 0.5, FIELD_LENGTH, FIELD_WIDTH)
        plot.addItem(img)

        # Initialize scatter plot for particles
        self.scatter = pg.ScatterPlotItem(size=self.num_particles)
        self.scatter.setData(size=1, pen=pg.mkPen(None))
        plot.addItem(self.scatter)

        # Initialize arrow for best estimate
        self.arrow = CenteredArrowItem()
        plot.addItem(self.arrow)

        # Initialize robot plotting of best estimate
        self.robot = pg.PlotCurveItem(
            ROBOT_COORDS[0, :],
            ROBOT_COORDS[1, :],
            antialias=True,
            pen=pg.mkPen("k", width=4),
        )
        plot.addItem(self.robot)

        self.ll_top = pg.PlotDataItem(
            [LL_CORDS[0, 0], LL_CORDS[0, 1]],
            [LL_CORDS[1, 0], LL_CORDS[1, 1]],
            antialias=True,
            pen=LL_FOV_PEN,
        )
        plot.addItem(self.ll_top)

        self.ll_bottom = pg.PlotDataItem(
            [LL_CORDS[0, 0], LL_CORDS[0, 2]],
            [LL_CORDS[1, 0], LL_CORDS[1, 2]],
            antialias=True,
            pen=LL_FOV_PEN,
        )
        plot.addItem(self.ll_bottom)

        self.ll_fill = pg.FillBetweenItem(
            self.ll_bottom, self.ll_top, brush=LL_FILL_BRUSH
        )
        self.ll_fill.setZValue(1)
        plot.addItem(self.ll_fill)

        self.ll_bearing = pg.PlotDataItem(
            [LL_BEARING_COORDS[0, 0], LL_BEARING_COORDS[0, 1]],
            [LL_BEARING_COORDS[1, 0], LL_BEARING_COORDS[1, 1]],
            antialias=True,
            pen=BEARING_PEN,
        )
        self.ll_bearing.setZValue(2)
        plot.addItem(self.ll_bearing)

        timer = QtCore.QTimer()
        timer.timeout.connect(self._update)
        timer.start(int(1000.0 / 60.0))

        pg.exec()

    def start(self):
        self.q = Queue()
        self.p = Process(target=self.run)
        self.p.start()
        return self.q

    def join(self):
        return self.p.join()

    def _update(self):
        # Fast fowrard to the latest element
        latest = None
        while not self.q.empty():
            latest = self.q.get()

        if latest is not None:
            vision, particles, best_estimate, has_vision = latest

            # Particles
            if particles is not None:
                NUM_TO_PLOT = 1000
                self.scatter.setData(
                    x=particles[:NUM_TO_PLOT, 0, 3],
                    y=particles[:NUM_TO_PLOT, 1, 3],
                    brush="g" if has_vision else "r",
                )

            x, y, theta = best_estimate

            # Update arrow
            self.arrow.setPos(x, y)
            self.arrow.setStyle(angle=-np.rad2deg(theta) + 180)

            # Update robot
            tr = QtGui.QTransform()
            tr.translate(x, y)
            tr.rotateRadians(theta)
            m = pg.transformToArray(tr)
            coords = np.dot(m, ROBOT_COORDS)
            self.robot.setData(coords[0, :], coords[1, :])

            # Draw LL FOV
            if has_vision:
                self.ll_top.setPen(LL_FOV_PEN)
                self.ll_bottom.setPen(LL_FOV_PEN)
                self.ll_fill.setBrush(LL_FILL_BRUSH)

                tr = QtGui.QTransform()
                tr.translate(x, y)
                tr.rotateRadians(theta)
                m = pg.transformToArray(tr)
                coords = np.dot(m, LL_CORDS)
                self.ll_top.setData(
                    [coords[0, 0], coords[0, 1]], [coords[1, 0], coords[1, 1]]
                )
                self.ll_bottom.setData(
                    [coords[0, 0], coords[0, 2]], [coords[1, 0], coords[1, 2]]
                )
            else:
                self.ll_top.setPen(pg.mkPen(color=pg.mkColor(0, 0, 0, 0)))
                self.ll_bottom.setPen(pg.mkPen(color=pg.mkColor(0, 0, 0, 0)))
                self.ll_fill.setBrush(pg.mkBrush(0, 0, 0, 0))

            # Draw measurement bearing
            if vision:
                self.ll_bearing.setPen(BEARING_PEN)

                tr = QtGui.QTransform()
                tr.translate(x, y)
                tr.rotateRadians(theta + vision[0].bearing)
                m = pg.transformToArray(tr)
                coords = np.dot(m, LL_BEARING_COORDS)
                self.ll_bearing.setData(
                    [coords[0, 0], coords[0, 1]], [coords[1, 0], coords[1, 1]]
                )
            else:
                self.ll_bearing.setPen(pg.mkPen(color=pg.mkColor(0, 0, 0, 0)))
