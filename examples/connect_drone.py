import time

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection

if __name__ == "__main__":
    conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
    # conn = WebSocketConnection('ws://127.0.0.1:5760')
    drone = Drone(conn)
    time.sleep(2)
    drone.start()
