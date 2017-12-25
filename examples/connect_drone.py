import time

from fcnd_drone_api import Drone

if __name__ == "__main__":
    drone = Drone(threaded=False, tlog_name="TLog-manual.txt")
    time.sleep(2)
    drone.start()
