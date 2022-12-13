import csv
from multiprocessing import Process, Manager, Queue
import pickle


class Logger:
    def __init__(self, filename):
        self.filename = filename
        csvlogfile = open(self.filename + ".csv", "w", newline="")
        self.writer = csv.writer(csvlogfile)
        self.logfile = open(self.filename + ".log", "ab")

    def init(self, odometry):
        pickle.dump(odometry, self.logfile)

    def log(self, time_elapsed, odometry, vision, best_estimate, has_vision):
        # Write simple data to CSV
        try:
            x, y, theta = best_estimate
            self.writer.writerow([time_elapsed, x, y, theta, has_vision])
        except Exception as e:
            print(e)

        # Write full data to log
        try:
            pickle.dump(
                (time_elapsed, odometry, vision, best_estimate, has_vision),
                self.logfile,
            )
        except Exception as e:
            print(e)
