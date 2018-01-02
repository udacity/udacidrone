import time

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection

if __name__ == "__main__":
    conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
    drone = Drone(conn, tlog_name="TLog-manual.txt")
    time.sleep(2)
    drone.start()
