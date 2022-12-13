from collections import deque
import logging
from networktables import NetworkTables
import numpy as np

from data_utils import Odometry, Vision

logger = logging.getLogger("NetworkTablesManager")
logging.basicConfig(level=logging.INFO)
KEY = "0"

class NTManager:
    def __init__(self, server=None):
        logger.info("Initializing NetworkTables...")
        NetworkTables.initialize(server)
        quixsam_table = NetworkTables.getTable("quixsam")

        # Table accessors
        self.odometry_table = quixsam_table.getSubTable("odometry")
        self.vision_table = quixsam_table.getSubTable("vision")
        self.estimates_table = quixsam_table.getSubTable("estimates")

        # Clear tables
        self.clear_table(self.odometry_table)
        self.clear_table(self.vision_table)
        self.clear_table(self.estimates_table)

        # Buffers to read data into
        self.last_odometry_id = -1  # Enforce IDs be always increasing
        self.odometry_buffer = deque()
        self.last_vision_id = -1
        self.vision_buffer = deque()

        # Setup listeners
        self.odometry_table.addEntryListener(self.odometry_received)
        self.vision_table.addEntryListener(self.vision_received)

    def is_connected(self):
        return NetworkTables.isConnected()

    def clear_table(self, table):
        for key in table.getKeys():
            table.delete(key)

    def odometry_received(self, table, key, value, is_new):
        try:
            id_ = int(value[0])
            if id_ <= self.last_odometry_id:
                logging.critical(f"Odometry ID decreased: {key}")
            elif np.isfinite(id_) and all(np.isfinite(value)) and len(value) == 7:
                self.odometry_buffer.append(
                    Odometry(
                        id_, value[1], value[2], value[3],
                    )
                )
                self.latest_odometry_id = id_
            else:
                logging.critical(f"Invalid odometry data: {key}: {value}")
        except ValueError:
            logging.critical(f"Invalid odometry data: {key}")

    def vision_received(self, table, key, value, is_new):
        try:
            id_ = int(value[0])
            if id_ <= self.last_vision_id:
                logging.critical(f"Vision ID decreased: {key}")
            elif np.isfinite(id_) and all(np.isfinite(value)) and len(value) == 5:
                self.vision_buffer.append(
                    [
                        Vision(id_, value[1], value[2]),
                    ]
                )
                self.latest_vision_id = id_
            else:
                logging.critical(f"Invalid vision data: {key}: {value}")
        except ValueError:
            logging.critical(f"Invalid vision data: {key}")

    def get_next(self):
        logging.debug(f"Buffer sizes: {len(self.odometry_buffer)}, {len(self.vision_buffer)}")

        next_odom = None
        next_vision = None
        if len(self.odometry_buffer) > 0:
            # Return the next odometry if it exists.
            next_odom = self.odometry_buffer.popleft()
            self.odometry_table.delete(str(next_odom.id))

            # Return the the next vision if it exists and has the same ID as odometry.
            # Flush all vision older than this odom message.
            while (
                len(self.vision_buffer) > 0
                and self.vision_buffer[0][0].id <= next_odom.id
            ):
                next_vision = self.vision_buffer.popleft()
                self.vision_table.delete(str(next_vision[0].id))
        return next_odom, next_vision

    def publish_estimate(self, id_: int, x: float, y: float, theta: float):
        self.estimates_table.putNumberArray(KEY, [float(id_), x, y, theta])
