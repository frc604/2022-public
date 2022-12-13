import argparse
import time
import pickle

from particle_filter import ParticleFilter
from plotter import Plotter

NUM_PARTICLES = 5000
IS_RED = True


def run(filename, speed):
    with open(filename, "rb") as logfile:
        plotter = Plotter(NUM_PARTICLES, is_red=IS_RED)
        plotter_queue = plotter.start()
        time.sleep(5.0)  # Wait for plotter to load

        init_odometry = pickle.load(logfile)
        pf = ParticleFilter(NUM_PARTICLES, init_odometry)

        rows = []
        while True:
            try:
                rows.append(pickle.load(logfile))
            except EOFError:
                break

        log_start_time = rows[0][0]
        start_time = time.perf_counter()
        for t, odometry, vision, log_best_estimate, _ in rows:
            if speed is not None:
                log_elapsed_time = t - log_start_time
                print(f"Elapsed time: {log_elapsed_time}")
                while time.perf_counter() - start_time < log_elapsed_time / speed:
                    pass

            pf.predict(odometry)
            has_vision = pf.update(vision)
            best_estimate = pf.get_best_estimate()
            plotter_queue.put((vision, pf.particles, best_estimate, has_vision))
            if log_best_estimate != best_estimate:
                dx = log_best_estimate[0] - best_estimate[0]
                dy = log_best_estimate[1] - best_estimate[1]
                dtheta = log_best_estimate[2] - best_estimate[2]
                error = dx, dy, dtheta
                print(f"Not reproducing log! Error: {error}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("logfile", help="Path to the log file")
    parser.add_argument("-s", type=float, help="Playback speed multiplier")
    args = parser.parse_args()
    run(args.logfile, args.s)
