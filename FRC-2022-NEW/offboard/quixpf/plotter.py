from turtle import title
import matplotlib.pyplot as plt
from multiprocessing import Process, Manager, Queue
import os
import numpy as np
from pyqtgraph.Qt import QtCore, QtGui
import transformations as tf
import pyqtgraph as pg

from data_utils import *
from helpers import in2m
from particle_filter import ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH, ROBOT_TO_CAMERA

FIELD_IMAGE = os.path.join(os.path.dirname(__file__), "../../field.png")

FIELD_LENGTH = in2m(12 * 54)  # m
FIELD_WIDTH = in2m(12 * 27)  # m


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
BEARING_RETRO_PEN = pg.mkPen(color=pg.mkColor(63, 191, 63, 255), width=4)
BEARING_TAG_PEN = pg.mkPen(color=pg.mkColor(255, 0, 255, 255), width=2)


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
        self.plot = self.win.addPlot()
        self.plot.setAspectLocked()

        # Show field
        img = pg.ImageItem()
        img.setImage(np.rot90(plt.imread(FIELD_IMAGE), k=3 if self.is_red else 1))
        img.setRect(-FIELD_LENGTH * 0.5, -FIELD_WIDTH * 0.5, FIELD_LENGTH, FIELD_WIDTH)
        self.plot.addItem(img)

        # Initialize scatter plot for targets
        self.targets = pg.ScatterPlotItem()
        self.targets.setData(
            size=0.1,
            brush=pg.mkColor(255, 0, 255, 255),
            pxMode=False
        )
        self.plot.addItem(self.targets)

        # Initialize scatter plot for particles
        self.scatter = pg.ScatterPlotItem(size=self.num_particles)
        self.scatter.setData(size=1, pen=pg.mkPen(None))
        self.plot.addItem(self.scatter)

        # Initialize arrow for best estimate
        self.arrow = CenteredArrowItem()
        self.plot.addItem(self.arrow)

        # Initialize robot plotting of best estimate
        self.robot = pg.PlotCurveItem(
            ROBOT_COORDS[0, :],
            ROBOT_COORDS[1, :],
            antialias=True,
            pen=pg.mkPen("k", width=4),
        )
        self.plot.addItem(self.robot)

        self.ll_top = pg.PlotDataItem(
            [LL_CORDS[0, 0], LL_CORDS[0, 1]],
            [LL_CORDS[1, 0], LL_CORDS[1, 1]],
            antialias=True,
            pen=LL_FOV_PEN,
        )
        self.plot.addItem(self.ll_top)

        self.ll_bottom = pg.PlotDataItem(
            [LL_CORDS[0, 0], LL_CORDS[0, 2]],
            [LL_CORDS[1, 0], LL_CORDS[1, 2]],
            antialias=True,
            pen=LL_FOV_PEN,
        )
        self.plot.addItem(self.ll_bottom)

        self.ll_fill = pg.FillBetweenItem(
            self.ll_bottom, self.ll_top, brush=LL_FILL_BRUSH
        )
        self.ll_fill.setZValue(1)
        self.plot.addItem(self.ll_fill)

        # By target id
        self.ll_bearings = {}

        timer = QtCore.QTimer()
        timer.timeout.connect(self._update)
        timer.start(int(1000.0 / 60.0))

        pg.exec()

    def start(self):
        self.q = Queue()
        self.targets_q = Queue()
        self.p = Process(target=self.run)
        self.p.start()
        return self.q, self.targets_q

    def join(self):
        return self.p.join()

    def _update(self):
        # Update targets
        latest_targets = None
        while not self.targets_q.empty():
            targets = np.array(list(self.targets_q.get().values()))
            self.targets.setData(
                x=targets[:, 0],
                y=targets[:, 1],
            )

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
            detected_ids = set()
            if vision:
                for v in vision:
                    # Add if we haven't seen this target ID before
                    if v.targetID not in self.ll_bearings:
                        self.ll_bearings[v.targetID] = pg.PlotDataItem(
                            [LL_BEARING_COORDS[0, 0], LL_BEARING_COORDS[0, 1]],
                            [LL_BEARING_COORDS[1, 0], LL_BEARING_COORDS[1, 1]],
                            antialias=True,
                        )
                        self.ll_bearings[v.targetID].setZValue(2)
                        self.plot.addItem(self.ll_bearings[v.targetID])

                    # Update ID
                    detected_ids.add(v.targetID)
                    self.ll_bearings[v.targetID].setPen(
                        BEARING_RETRO_PEN if v.targetID == -1 else BEARING_TAG_PEN)

                    tr = QtGui.QTransform()
                    tr.translate(x, y)
                    tr.rotateRadians(theta + v.bearing)
                    m = pg.transformToArray(tr)
                    coords = np.dot(m, LL_BEARING_COORDS)
                    self.ll_bearings[v.targetID].setData(
                        [coords[0, 0], coords[0, 1]], [coords[1, 0], coords[1, 1]]
                    )
            # Turn off all unseen ids
            for targetID, item in self.ll_bearings.items():
                if targetID not in detected_ids:
                    item.setPen(pg.mkPen(color=pg.mkColor(0, 0, 0, 0)))
