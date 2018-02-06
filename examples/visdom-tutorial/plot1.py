import argparse

import numpy as np
import visdom

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID


class MyDrone(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        # default opens up to http://localhost:8097
        self.v = visdom.Visdom()
        assert self.v.check_connection()

        # Plot NE
        ne = np.array([self.local_position[0], self.local_position[1]]).reshape(-1, 2)
        self.ne_plot = self.v.scatter(
            ne, opts=dict(title="Local position (north, east)", xlabel='North', ylabel='East'))

        # Plot D
        d = np.array([self.local_position[2]])
        self.t = 0
        self.d_plot = self.v.line(
            d, X=np.array([self.t]), opts=dict(title="Altitude (meters)", xlabel='Timestep', ylabel='Down'))

        self.register_callback(MsgID.LOCAL_POSITION, self.update_ne_plot)
        self.register_callback(MsgID.LOCAL_POSITION, self.update_d_plot)

    def update_ne_plot(self):
        ne = np.array([self.local_position[0], self.local_position[1]]).reshape(-1, 2)
        self.v.scatter(ne, win=self.ne_plot, update='append')

    def update_d_plot(self):
        d = np.array([self.local_position[2]])
        self.t += 1
        self.v.line(d, X=np.array([self.t]), win=self.d_plot, update='append')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    drone = MyDrone(conn)
    drone.start()


if __name__ == '__main__':
    main()
