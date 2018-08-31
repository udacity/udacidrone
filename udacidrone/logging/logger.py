"""class to handling logging data to a file

class and helper methods for logging both telemetry and custom data to a file.
"""

import os
import queue
import threading
import time

from enum import Enum
import numpy as np


class LogType(Enum):
    """type of data that is being passed to be logged"""
    RAW_DATA = 1
    TELEMETRY_MSG = 2


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

        # variables needed for handling the logging tasks on a background thread
        self._log_data_queue = queue.Queue()  # a queue for sending data between threads
        write_handle = threading.Thread(target=self._logging_loop)
        write_handle.daemon = True

        # immediately start the loop
        write_handle.start()

    def close(self):
        """close the log (and the corresponding log file) and stop logging"""
        self.open = False
        time.sleep(0.1)
        self.log.close()

    def log_data(self, data):
        """log data to a file

        save a custom set of data to the log file.
        note that it is recommended that there is a marker in the data list to identify
        what type of data is being logged in the case of logging many custom messages.

        the data is added to the queue to be handled by the file writing background thread
        as a tuple of (TYPE, data)

        Arguments:
            data {list} -- the list of items to save as a csv row in the log file.
        """

        # need to make sure that the data is what is expected
        # TODO: this forces all data entries to be the same length...
        # TODO: may want to allow variable length custom messages
        if (self.num_data == 0):
            self.num_data = len(data)

        if (len(data) != self.num_data):
            print("Logger: Invalid number of entries")
            return

        # add the data to the queue of messages that need to be logged
        # note data is passed in as a tuple (TYPE, data)
        self._log_data_queue.put((LogType.RAW_DATA, data))

    def log_telemetry_msg(self, msg_name, msg):
        """log a telemetry message

        save an incoming telemetry message as a csv row in the log file.
        this will adjust the data to prepend the message name to the start of the row
        to help identify the contents of the row.

        the data is added to the queue to be handled by the file writing background thread
        as a tuple of (TYPE, (msg_name, msg))

        Arguments:
            msg_name {string} -- the name of the message
            msg {Message} -- the raw telemetry message from the connection
        """

        # add the data to the queue of messages that need to be logged
        # note data is passed in as a tuple (TYPE, data)
        self._log_data_queue.put((LogType.TELEMETRY_MSG, (msg_name, msg)))

    def _write_data_to_log(self, log_data):
        """helper function to take the log data tuple and write the csv row

        handles the actually writing of the data to the file.
        this is called by the background thread.
        telemetry messages need to be formatted into a list before writing to the csv file
        so based on the message type, the data is handled accordingly.

        see the LogType enum above to know what types this function can handle

        Arguments:
            log_data {tuple} -- the log data as a tuple (LogType, data)
        """

        # make sure there is data present
        if log_data is None:
            return

        # split the tuple to see if data needs to get adjusted
        log_type, data = log_data

        # the data for a telemetry message comes in as a tuple
        # and needs to be formatted properly to a list to be logged
        if log_type == LogType.TELEMETRY_MSG:
            msg_name, msg = data
            temp_data = [msg_name]
            temp_data.append(msg.time)
            for k in msg.__dict__.keys():
                if k != '_time':
                    temp_data.append(msg.__dict__[k])
            data = temp_data

        # write the data (in an list form) to the log file
        num_elements = len(data)
        for i in range(num_elements):
            d = data[i]
            if isinstance(d, float):
                self.log.write('{0:.7f}'.format(d))
            else:
                self.log.write(d.__str__())
            if i != num_elements - 1:
                self.log.write(',')
        self.log.write('\n')

    def _logging_loop(self):
        """loop to be run in a background thread to handle the actually writing to a file.

        takes the log data in the queue and writes the data to a csv file.
        this is designed to be run as a background thread so the "heavy lifting"
        is not done on the same thread as the callbacks for the controllers.
        """

        # continue to loop until this connection is closed
        while self.open:

            # go through all the queued messages and log them
            while not self._log_data_queue.empty():
                try:
                    log_data = self._log_data_queue.get_nowait()
                except queue.Empty:
                    # if there is no msgs in the queue, will just continue
                    pass
                else:
                    # write the list data to the log file
                    self._write_data_to_log(log_data)

            # TODO: could rate limit this loop if needed...


def read_log(filename):
    """
    Returns a numpy 2D array of the data
    """
    return np.loadtxt(filename, delimiter=',', dtype='Float64')
