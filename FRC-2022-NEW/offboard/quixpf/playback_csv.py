import csv
import time

from plotter import Plotter


def run():
    with open("quixsamlog_example.csv") as logfile:
        plotter = Plotter(0)
        plotter_queue = plotter.start()

        reader = csv.reader(logfile)
        for t, x, y, theta, has_vision in reader:
            estimate = (float(x), float(y), float(theta))
            plotter_queue.put((None, None, estimate, has_vision == "True"))
            time.sleep(0.01)


if __name__ == "__main__":
    run()
