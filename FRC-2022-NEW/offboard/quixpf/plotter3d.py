import matplotlib.pyplot as plt
from multiprocessing import Process, Manager, Queue
import numpy as np
import os
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore
from stl import mesh

from helpers import in2m, besph2cart
from particle_filter import ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH, ROBOT_TO_CAMERA, TARGET_CENTER_X, TARGET_CENTER_Y, TARGET_CENTER_Z, TARGET_RADIUS, ROBOT_TO_CAMERA

FIELD_IMAGE = os.path.join(os.path.dirname(__file__), "../../field.png")
FIELD_LENGTH = in2m(12 * 54)  # m
FIELD_WIDTH = in2m(12 * 27)  # m

FIELD_MESH = mesh.Mesh.from_file(os.path.join(os.path.dirname(__file__), "../../field.stl"))
ROBOT_MESH = mesh.Mesh.from_file(os.path.join(os.path.dirname(__file__), "../../robot.stl"))

ROBOT_LINES = np.array([
    # Edges
    [ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH, 0],
    [-ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH, 0],
    [-ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH, 0],
    [-ROBOT_HALF_LENGTH, -ROBOT_HALF_WIDTH, 0],
    [-ROBOT_HALF_LENGTH, -ROBOT_HALF_WIDTH, 0],
    [ROBOT_HALF_LENGTH, -ROBOT_HALF_WIDTH, 0],
    [ROBOT_HALF_LENGTH, -ROBOT_HALF_WIDTH, 0],
    [ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH, 0],
    # Arrow
    [-0.1, 0.1, 0],
    [0.2, 0.0, 0],
    [0.2, 0.0, 0],
    [-0.1, -0.1, 0],
    [-0.1, -0.1, 0],
    [-0.1, 0.1, 0],
])

NUM_RING_POINTS = 20
RING_POINTS = []
for i in range(NUM_RING_POINTS):
    rad = i * (2.0 * np.pi / NUM_RING_POINTS)
    x = TARGET_RADIUS * np.cos(rad)
    y = TARGET_RADIUS * np.sin(rad)
    RING_POINTS.append((x, y, TARGET_CENTER_Z))
RING_POINTS.append(RING_POINTS[0])  # Close the circle

class Plotter3d:
    def __init__(self, num_particles, is_red=True):
        self.num_particles = num_particles
        self.is_red = is_red

    def start(self):
        self.q = Queue()
        self.targets_q = Queue()
        self.p = Process(target=self.run)
        self.p.start()
        return self.q, self.targets_q

    def join(self):
        return self.p.join()

    def run(self):
        app = pg.mkQApp("QuixPF")
        self.w = gl.GLViewWidget()
        self.w.show()
        self.w.orbit(210, 0)
        self.w.setCameraPosition(distance=20)

        ax = gl.GLAxisItem()
        ax.setSize(1, 1, 1)
        self.w.addItem(ax)

        grid = gl.GLGridItem()
        grid.setSize(18, 10)
        grid.setSpacing(1, 1)
        self.w.addItem(grid)

        field_img = plt.imread(FIELD_IMAGE)
        scale = FIELD_LENGTH / field_img.shape[1]
        image_item = gl.GLImageItem(pg.makeRGBA(field_img, levels=(0, 1))[0])
        image_item.scale(scale, scale, scale)
        image_item.translate(-FIELD_WIDTH * 0.5, -FIELD_LENGTH * 0.5, 0)
        image_item.rotate(-90, 0, 0, 1)
        image_item.setGLOptions("opaque")
        self.w.addItem(image_item)

        # points = FIELD_MESH.points.reshape(-1, 3)
        # faces = np.arange(points.shape[0]).reshape(-1, 3)
        # mesh_data = gl.MeshData(vertexes=points, faces=faces)
        # field_mesh = gl.GLMeshItem(
        #     meshdata=mesh_data,
        #     smooth=True,
        #     drawFaces=True,
        #     drawEdges=True,
        #     color=(0.5, 0.5, 0.5, 1),
        #     edgeColor=(0, 0, 0, 1)
        # )
        # field_mesh.translate(0, 0, -0.01)
        # self.w.addItem(field_mesh)

        ring = gl.GLLinePlotItem(
            pos=RING_POINTS,
            width=5,
            color=(0, 1, 0, 1),
        )
        ring.setGLOptions("opaque")
        self.w.addItem(ring)
        
        self.targets = gl.GLScatterPlotItem(color=(1, 0, 1, 1), size=0.1, pxMode=False)
        self.targets.setGLOptions("opaque")
        self.w.addItem(self.targets)

        points = ROBOT_MESH.points.reshape(-1, 3)
        faces = np.arange(points.shape[0]).reshape(-1, 3)
        mesh_data = gl.MeshData(vertexes=points, faces=faces)
        self.robot_mesh = gl.GLMeshItem(
            meshdata=mesh_data,
            smooth=True,
            drawFaces=True,
            drawEdges=True,
            color=(0.5, 0.5, 0.5, 1),
            edgeColor=(0, 0, 0, 1)
        )
        self.robot_mesh.scale(1e-3, 1e-3, 1e-3)
        self.w.addItem(self.robot_mesh)

        self.robot = gl.GLLinePlotItem(
            pos=ROBOT_LINES,
            width=5,
            color=(0, 0, 0, 1),
            mode="lines"
        )
        self.robot.setGLOptions("opaque")
        self.w.addItem(self.robot)

        self.particles = gl.GLScatterPlotItem(size=2)
        self.w.addItem(self.particles)

        # By target ID
        self.rays = {}

        timer = QtCore.QTimer()
        timer.timeout.connect(self._update)
        timer.start(int(1000.0 / 60.0))

        pg.exec()

    def _update(self):
        # Update targets
        latest_targets = None
        while not self.targets_q.empty():
            targets = self.targets_q.get()
            self.targets.setData(pos=list(targets.values()))
        
        # Fast fowrard to the latest element
        latest = None
        while not self.q.empty():
            latest = self.q.get()

        if latest is not None:
            vision, particles, best_estimate, has_vision = latest

            # Particles
            if particles is not None:
                NUM_TO_PLOT = 5000
                pos = np.zeros((NUM_TO_PLOT, 3))
                pos[:, 0] = particles[:NUM_TO_PLOT, 0, 3]
                pos[:, 1] = particles[:NUM_TO_PLOT, 1, 3]
                self.particles.setData(
                    pos=pos,
                    color=(0, 1, 0, 1) if has_vision else (1, 0, 0, 1)
                )

            # Robot
            x, y, theta = best_estimate
            field_robot_T = pg.Transform3D()
            field_robot_T.translate(x, y, 0.05)
            field_robot_T.rotate(np.rad2deg(theta), 0, 0, 1)
            self.robot.setTransform(field_robot_T)

            self.robot_mesh.setTransform(field_robot_T)
            self.robot_mesh.scale(1e-3, 1e-3, 1e-3)
            self.robot_mesh.rotate(90, 0, 0, 1, local=True)

            # Measurement rays
            detected_ids = set()
            if vision:
                for v in vision:
                    # Add if we haven't seen this target ID before
                    if v.targetID not in self.rays:
                        self.rays[v.targetID] = gl.GLLinePlotItem(
                            mode="lines")
                        self.rays[v.targetID].setGLOptions("opaque")
                        self.w.addItem(self.rays[v.targetID])

                    # Update ID
                    detected_ids.add(v.targetID)
                    isRetro = v.targetID == -1

                    ray_length = in2m(25 * 12) if isRetro else in2m(15 * 12)
                    self.rays[v.targetID].setTransform(field_robot_T * pg.Transform3D(ROBOT_TO_CAMERA))
                    self.rays[v.targetID].setData(
                        pos=np.array([[0, 0, 0], ray_length * besph2cart(v.bearing, v.elevation)]),
                        color=(0, 1, 0, 1) if isRetro else (1, 0, 1, 1),
                        width=4 if isRetro else 2
                    )
                    self.rays[v.targetID].setGLOptions("additive")
            # Turn off all unseen IDs
            for targetID, item in self.rays.items():
                if targetID not in detected_ids:
                    item.setGLOptions("additive")
                    item.setData(color=(0, 0, 0, 0))


if __name__ == "__main__":
    plotter = Plotter3d(5000)
    plotter_queue = plotter.start()
