from datetime import datetime
import logging
from nt_manager import NTManager
from multiprocessing import Process
from particle_filter import ParticleFilter
from plotter import Plotter
from logger import Logger
import time

NUM_PARTICLES = 5000


class Quixsam:
    def __init__(self, server=None):
        self.server = server

    def run(self):
        # Start plotter thread
        plotter = Plotter(NUM_PARTICLES)
        plotter_queue = plotter.start()

        # Continuously reset and try to reconnect if disconnected
        while True:
            nt_manager = NTManager(self.server)

            pf = None
            start_time = None
            logger = None
            while pf is None:
                logging.info("Waiting for first odometry...")
                odometry, _ = nt_manager.get_next()
                if odometry is not None:
                    pf = ParticleFilter(NUM_PARTICLES, odometry)
                    logging.info("First odometry received!")
                    start_time = time.perf_counter()

                    # Start new logger every time we connect.
                    try:
                        now_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                        logger = Logger(f"quixsamlog_{now_str}")
                        logger.init(odometry)
                    except Exception as e:
                        print("Logger failed to start")
                        print(e)
                else:
                    time.sleep(0.1)

            while nt_manager.is_connected():
                odometry, vision = nt_manager.get_next()
                if odometry is not None:
                    s = time.perf_counter()
                    pf.predict(odometry)
                    has_vision = pf.update(vision)

                    # Publish estimate
                    x, y, theta = pf.get_best_estimate()
                    nt_manager.publish_estimate(odometry.id, x, y, theta)

                    # Log for resimulation
                    if logger is not None:
                        try:
                            logger.log(
                                s - start_time,
                                odometry,
                                vision,
                                pf.get_best_estimate(),
                                has_vision,
                            )
                        except Exception as e:
                            print("Log failed.")

                    # Handle plotting
                    plotter_queue.put(
                        (vision, pf.particles, pf.get_best_estimate(), has_vision)
                    )
                    if odometry.id % 5 == 0:
                        logging.info(
                            "Iter {} time: {:.3f} {}".format(
                                odometry.id,
                                time.perf_counter() - s,
                                "VISION" if has_vision else "",
                            )
                        )
                time.sleep(0.001)


if __name__ == "__main__":
    Quixsam("10.6.4.2").run()
