"""
Plot streaming data. Requires realtime plotting library `visdom`, https://github.com/facebookresearch/visdom/


"""

from collections import deque
import numpy as np
import visdom

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID


class MyDrone(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.v = visdom.Visdom(server='http://localhost', port=3003)
        assert self.v.check_connection()

        # plot local position

        # create q and initial point
        self.local_position_q = deque(maxlen=400)
        self.local_position_q.append([self.local_position[0], self.local_position[1]])

        # turn queue into a numpy array
        X = np.array(self.local_position_q).reshape(-1, 2)
        self.local_position_plot = self.v.scatter(X, opts=dict(title="Local position (x, y)", xlabel='X', ylabel='Y'))

        # plot altitude (meters)

        self.altitude_q = deque(maxlen=400)
        self.altitude_q.append(self.local_position[2])
        self.altitude_timestep_q = deque(maxlen=400)
        self.altitude_timestep_q.append(0)

        Y = np.array(self.altitude_q)
        X = np.array(self.altitude_timestep_q)
        self.altitude_plot = self.v.line(Y, X=X, opts=dict(title="Altitude (meters)", xlabel='Timestep'))

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
    conn = MavlinkConnection('tcp:127.0.0.1:5760')
    drone = MyDrone(conn)
    drone.start()


if __name__ == '__main__':
    main()
