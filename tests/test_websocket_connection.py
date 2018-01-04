"""
"""
import time

from udacidrone import Drone
from udacidrone.connection import WebSocketConnection

if __name__ == "__main__":
    conn = WebSocketConnection('ws://127.0.0.1:5760')
    drone = Drone(conn, tlog_name="TLog-manual.txt")
    time.sleep(2)
    drone.start()
