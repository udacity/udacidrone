"""
Example plotting realtime drone data using `visdom`, https://github.com/facebookresearch/visdom/

1. Start visdom server `python -m visdom.server` and go to the link.
This is where the plots will appear.
2. Start the simulator.
3. Run this script.
4. Fly around in the simulator manually. You should see the local position and altitude plots updating.
"""

import argparse
from collections import deque

import numpy as np
import visdom

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID


class MyDrone(Drone):

    def __init__(self, connection, qsize):
        super().__init__(connection)
        # default opens up to http://localhost:8097
        self.v = visdom.Visdom()
        assert self.v.check_connection()
        self.qsize = qsize

        # plot local position

        # create q and initial point
        self.local_position_q = deque(maxlen=self.qsize)
        self.local_position_q.append([self.local_position[0], self.local_position[1]])

        # turn queue into a numpy array
        X = np.array(self.local_position_q).reshape(-1, 2)
        self.local_position_plot = self.v.scatter(
            X, opts=dict(title="Local position (north, east)", xlabel='North', ylabel='East'))

        # plot altitude (meters)

        self.altitude_q = deque(maxlen=self.qsize)
        self.altitude_q.append(self.local_position[2])
        self.altitude_timestep_q = deque(maxlen=self.qsize)
        self.altitude_timestep_q.append(1)

        Y = np.array(self.altitude_q)
        X = np.array(self.altitude_timestep_q)
        self.altitude_plot = self.v.line(Y, X=X, opts=dict(title="Altitude (meters)", xlabel='Timestep', ylabel='Down'))

        # register plotting callbacks
        self.register_callback(MsgID.LOCAL_POSITION, self.update_local_pos_plot_callback)
        self.register_callback(MsgID.LOCAL_POSITION, self.update_altitude_plot_callback)

    def update_local_pos_plot_callback(self):
        self.local_position_q.append([self.local_position[0], self.local_position[1]])
        X = np.array(self.local_position_q).reshape(-1, 2)
        self.v.scatter(X, win=self.local_position_plot, update='insert')

    def update_altitude_plot_callback(self):
        self.altitude_q.append(self.local_position[2])
        self.altitude_timestep_q.append(self.altitude_timestep_q[-1] + 1)
        Y = np.array(self.altitude_q)
        X = np.array(self.altitude_timestep_q)
        self.v.line(Y, X=X, win=self.altitude_plot, update='insert')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--qsize', type=int, default=200, help='Maximum number of points to plot at a given time')
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    drone = MyDrone(conn, args.qsize)
    drone.start()


if __name__ == '__main__':
    main()
