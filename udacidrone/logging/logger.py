import os

import numpy as np


class Logger(object):
    """"""

    def __init__(self, directory='Logs', filename='NavLog.txt'):
        filepath = os.path.join(directory, filename)
        print(filepath)
        if not os.path.exists(directory):
            os.makedirs(directory)

        self.log = open(filepath, 'w')
        self.num_data = 0
        self.open = True

    def close(self):
        self.log.close()
        self.open = False

    def log_data(self, data):
        if (self.num_data == 0):
            self.num_data = len(data)

        if (len(data) != self.num_data):
            print("Logger: Invalid number of entries")
            return

        for i in range(len(data)):
            if type(data[i]) == float:
                self.log.write('{0:.7f}'.format(data[i]))
            else:
                self.log.write(data[i].__str__())
            if i != len(data) - 1:
                self.log.write(',')

        self.log.write('\n')

    def log_telemetry_data(self, data):
        for i in range(len(data)):
            if type(data[i]) == float:
                self.log.write('{0:.7f}'.format(data[i]))
            else:
                self.log.write(data[i].__str__())
            if i != len(data) - 1:
                self.log.write(',')

        self.log.write('\n')


def read_log(filename):
    """
    Returns a numpy 2D array of the data
    """
    return np.loadtxt(filename, delimiter=',', dtype='Float64')
