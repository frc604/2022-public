import argparse
from datetime import datetime
import logging
from nt_manager import NTManager
from multiprocessing import Process
from particle_filter import ParticleFilter
from plotter import Plotter
from plotter3d import Plotter3d
from logger import Logger
import time

NUM_PARTICLES = 5000


class Quixsam:
    def __init__(self, server=None, save_logs=True, view3d=False):
        self.server = server
        self.save_logs = save_logs
        self.view3d = view3d

    def run(self):
        # Start plotter thread
        plotter = Plotter3d(NUM_PARTICLES) if self.view3d else Plotter(NUM_PARTICLES)
        plotter_queue, plotter_targets_queue = plotter.start()

        # Continuously reset and try to reconnect if disconnected
        while True:
            nt_manager = NTManager(self.server)

            pf = None
            start_time = None
            logger = None
            while pf is None:
                logging.info("Waiting for targets...")
                targets = nt_manager.get_targets()
                if not targets:
                    time.sleep(0.1)
                    continue

                plotter_targets_queue.put(targets)

                logging.info("Waiting for first odometry...")
                odometry, _ = nt_manager.get_next()
                if odometry is not None:
                    pf = ParticleFilter(NUM_PARTICLES, odometry, targets)
                    logging.info("First odometry received!")
                    start_time = time.perf_counter()

                    # Start new logger every time we connect.
                    if self.save_logs:
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
    parser = argparse.ArgumentParser()
    parser.add_argument('--local', action='store_true')
    parser.add_argument('--view3d', action='store_true')
    args = parser.parse_args()
    Quixsam("localhost" if args.local else "10.6.4.2", save_logs=not args.local, view3d=args.view3d).run()
