import traceback

import numpy as np

from udacidrone.logging import Logger
from udacidrone.messaging import MsgID


class Drone(object):
    """
    Drone class
    """

    def __init__(self, connection, tlog_name="TLog.txt"):
        self.connection = connection

        # Global position in degrees (int)
        # Altitude is in meters
        self._longitude = 0
        self._latitude = 0
        self._altitude = 0

        # Reference home position in degrees (int)
        # Altitude is in meters
        self._home_longitude = 0
        self._home_latitude = 0
        self._home_altitude = 0

        # Local positions in meters from the global home (float)
        # In NED frame
        self._north = 0.0
        self._east = 0.0
        self._down = 0.0

        # Locally oriented velocity in meters/second
        # In NED frame
        self._velocity_north = 0.0
        self._velocity_east = 0.0
        self._velocity_down = 0.0

        # If the drone is armed the motors are powered and the rotors are spinning.
        self._armed = False

        # If the drone is guided it is being autonomously controlled,
        # the other opposite would be manual control.
        self._guided = False

        # Euler angles in radians
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0

        # Drone body accelerations
        self._acceleration_x = 0.0
        self._acceleration_y = 0.0
        self._acceleration_z = 0.0

        # Drone gyro rates or angular velocities in radians/second
        # TODO: Explain what x, y, and z are.
        self._gyro_x = 0.0
        self._gyro_y = 0.0
        self._gyro_z = 0.0

        # Barometer
        # TODO: better explanation
        self._baro_altitude = 0.0

        self._update_property = {
            MsgID.STATE: self._update_state,
            MsgID.GLOBAL_POSITION: self._update_global_position,
            MsgID.LOCAL_POSITION: self._update_local_position,
            MsgID.GLOBAL_HOME: self._update_global_home,
            MsgID.LOCAL_VELOCITY: self._update_local_velocity,
            MsgID.RAW_GYROSCOPE: self._update_gyro_raw,
            MsgID.RAW_ACCELEROMETER: self._update_acceleration_raw,
            MsgID.BAROMETER: self._update_barometer,
            MsgID.ATTITUDE: self._update_attitude
        }

        # set the internal callbacks list to an empty dict
        self._callbacks = {}

        # configure this drone class to listen to all of the messages from the connection

        def on_message_receive(msg_name, msg):
            """Sorts incoming messages, updates the drone state variables and runs callbacks"""
            # print('Message received', msg_name, msg)
            if msg_name == MsgID.CONNECTION_CLOSED:
                self.stop()
            if msg_name in self._update_property.keys():
                self._update_property[msg_name](msg)
            self.notify_callbacks(msg_name, msg)  # pass it along to these listeners
            self.log_telemetry(msg_name, msg)

        # add the above callback function as a listener for all connection messages
        self.connection.add_message_listener(MsgID.ANY, on_message_receive)

        self.tlog = Logger("Logs", tlog_name)

    @property
    def global_position(self):
        return np.array([self._longitude, self._latitude, self._altitude])

    def _update_global_position(self, msg):
        self._longitude = msg.longitude
        self._latitude = msg.latitude
        self._altitude = msg.altitude

    @property
    def global_home(self):
        return np.array([self._home_longitude, self._home_latitude, self._home_altitude])

    def _update_global_home(self, msg):
        self._home_longitude = msg.longitude
        self._home_latitude = msg.latitude
        self._home_altitude = msg.altitude

    @property
    def local_position(self):
        return np.array([self._north, self._east, self._down])

    def _update_local_position(self, msg):
        self._north = msg.north
        self._east = msg.east
        self._down = msg.down

    @property
    def local_velocity(self):
        return np.array([self._velocity_north, self._velocity_east, self._velocity_down])

    def _update_local_velocity(self, msg):
        self._velocity_north = msg.north
        self._velocity_east = msg.east
        self._velocity_down = msg.down

    @property
    def armed(self):
        return self._armed

    @property
    def guided(self):
        return self._guided

    @property
    def connected(self):
        return self.connection.open

    def _update_state(self, msg):
        self._armed = msg.armed
        self._guided = msg.guided

    @property
    def attitude(self):
        """Roll, pitch, yaw euler angles in radians"""
        return np.array([self._roll, self._pitch, self._yaw])

    def _update_attitude(self, msg):
        self._roll = msg.roll
        self._pitch = msg.pitch
        self._yaw = msg.yaw

    @property
    def acceleration_raw(self):
        return np.array([self._acceleration_x, self._acceleration_y, self._acceleration_z])

    def _update_acceleration_raw(self, msg):
        self._acceleration_x = msg.x
        self._acceleration_y = msg.y
        self._acceleration_z = msg.z

    @property
    def gyro_raw(self):
        """Angular velocites in radians/second"""
        return np.array([self._gyro_x, self._gyro_y, self._gyro_z])

    def _update_gyro_raw(self, msg):
        self._gyro_x = msg.x
        self._gyro_y = msg.y
        self._gyro_z = msg.z

    @property
    def barometer(self):
        return np.array(self._baro_altitude)

    def _update_barometer(self, msg):
        self._baro_altitude = msg.altitude

    def log_telemetry(self, msg_name, msg):
        """Save the msg information to the telemetry log"""
        if self.tlog.open:
            data = [msg_name]
            data.append(msg.time)
            for k in msg.__dict__.keys():
                if k != '_time':
                    data.append(msg.__dict__[k])
            self.tlog.log_telemetry_data(data)

    @staticmethod
    def read_telemetry_data(filename):
        log_dict = {}
        tlog = open(filename, 'r')
        lines = tlog.readlines()

        for line in lines:
            line_split = line.rstrip('\n').split(',')
            if line_split[0] in log_dict.keys():
                entry = log_dict[line_split[0]]
                for i in range(1, len(line_split)):
                    if line_split[i] == 'True':
                        entry[i - 1] = np.append(entry[i - 1], True)
                    elif line_split[i] == 'False':
                        entry[i - 1] = np.append(entry[i - 1], False)
                    else:
                        entry[i - 1] = np.append(entry[i - 1], float(line_split[i]))
            else:
                entry = []
                for i in range(1, len(line_split)):
                    if line_split[i] == 'True':
                        entry.append(np.array(True))
                    elif line_split[i] == 'False':
                        entry.append(np.array(False))
                    else:
                        entry.append(np.array(float(line_split[i])))

            log_dict[line_split[0]] = entry
        return log_dict

    #
    # Handling of internal messages for callbacks
    #

    def register_callback(self, name, fn):
        """Add the function, `fn`, as a callback for the message type, `name`.

        Args:
            name: MsgID describing the message id
            fn: Callback function

        Example:

            self.add_message_listener(MsgID.GLOBAL_POSITION, global_msg_listener)

            OR

            self.add_message_listener(MsgID.ANY, all_msg_listener)

        These can be added anywhere in the code and are identical to initializing a callback with the decorator
        """
        if name not in self._callbacks:
            self._callbacks[name] = []
        if fn not in self._callbacks[name]:
            self._callbacks[name].append(fn)

    def remove_callback(self, name, fn):
        """Remove the function, `fn`, as a callback for the message type, `name`

        Args:
            name: MsgID describing the message id
            fn: Callback function

        Example:

            self.remove_message_listener(MsgID.GLOBAL_POSITION, global_msg_listener)

        """
        if name in self._callbacks:
            if fn in self._callbacks[name]:
                self._callbacks[name].remove(fn)
                if len(self._callbacks[name]) == 0:
                    del self._callbacks[name]

    def notify_callbacks(self, name, msg):
        """Passes the message to the appropriate listeners"""
        for fn in self._callbacks.get(name, []):
            try:
                # print('Drone executing {0} callback'.format(name))
                fn()
            except Exception as e:
                traceback.print_exc()

        for fn in self._callbacks.get(MsgID.ANY, []):
            try:
                print('Drone executing {0} callback'.format(MsgID.ANY))
                fn(name)
            except Exception as e:
                traceback.print_exc()

    #
    # Command method wrappers
    #

    def arm(self):
        """Send an arm command to the drone"""
        try:
            self.connection.arm()
        except Exception as e:
            traceback.print_exc()

    def disarm(self):
        """Send a disarm command to the drone"""
        try:
            self.connection.disarm()
        except Exception as e:
            traceback.print_exc()

    def take_control(self):
        """If the drone is in guided mode this will switch to manual mode"""
        print('Take Control Messsage')
        try:
            self.connection.take_control()
        except Exception as e:
            traceback.print_exc()

    def release_control(self):
        """Take control of the drone """
        try:
            self.connection.release_control()
        except Exception as e:
            traceback.print_exc()

    def cmd_position(self, north, east, down, heading):
        """Command the local position and drone heading.

        Args:
            north: local north in meters
            east: local east in meters
            down: local down in meters (positive down)
            heading: drone yaw in radians
        """
        try:
            self.connection.cmd_position(north, east, down, heading)
        except Exception as e:
            traceback.print_exc()

    def takeoff(self, target_altitude):
        """Command the drone to takeoff to the target_alt (in meters)"""
        try:
            self.connection.takeoff(self.local_position[0], self.local_position[1], target_altitude)
        except Exception as e:
            traceback.print_exc()

    def land(self):
        """Command the drone to land at its current position"""
        try:
            self.connection.land(self.local_position[0], self.local_position[1])
        except Exception as e:
            traceback.print_exc()

    def cmd_attitude(self, roll, pitch, yawrate, thrust):
        """Command the drone through attitude command

        Args:
            roll: in radians
            pitch: in randians
            yawrate: in radians/second
            thrust: upward acceleration in meters/second^2
        """
        try:
            self.connection.cmd_attitude(roll, pitch, yawrate, thrust)
        except Exception as e:
            traceback.print_exc()

    def cmd_attitude_rate(self, roll_rate, pitch_rate, yaw_rate, thrust):
        """Command the drone orientation rates.

        Args:
            roll_rate: in radians/second
            pitch_rate: in radians/second
            yaw_rate: in radians/second
            thrust: upward acceleration in meters/second^2
        """
        try:
            # thrust = np.clip(thrust, -1, 1)
            # yaw_rate = np.clip(yaw_rate, -1, 1)
            # roll_rate = np.clip(roll_rate, -1, 1)
            # pitch_rate = np.clip(pitch_rate, -1, 1)
            self.connection.cmd_attitude_rate(roll_rate, pitch_rate, yaw_rate, thrust)
        except Exception as e:
            traceback.print_exc()

    def cmd_moment(self, roll_moment, pitch_moment, yaw_moment, thrust):
        """Command the drone moments.

        Args:
            roll_moment: in Newton*meter
            pitch_moment: in Newton*meter
            yaw_moment: in Newton*meter
            thrust: upward force in Newtons
        """
        try:
            self.connection.cmd_moment(roll_moment, pitch_moment, yaw_moment, thrust)
        except Exception as e:
            traceback.print_exc()

    def cmd_velocity(self, velocity_north, velocity_east, velocity_down, heading):
        """Command the drone velocity.

        Args:
            north_velocity: in meters/second
            east_velocity: in meters/second
            down_velocity: in meters/second
            heading: in radians
        """
        try:
            self.connection.cmd_velocity(velocity_north, velocity_east, velocity_down, heading)
        except Exception as e:
            traceback.print_exc()

    def cmd_motors(self, motor_rpm):
        """Command the rmp of the motors"""
        try:
            self.connection.cmd_motors(motor_rpm)
        except Exception as e:
            traceback.print_exc()

    def set_home_position(self, longitude, latitude, altitude):
        """Set the drone's home position to these coordinates"""
        try:
            self.connection.set_home_position(latitude, longitude, altitude)

        except Exception as e:
            traceback.print_exc()

    def start_log(self, directory, name):
        self.log = Logger(directory, name)

    def stop_log(self):
        """Stop collection of logs"""
        self.log.close()

    def start(self):
        """Starts the connection to the drone"""

        # start the connection
        self.connection.start()

    def stop(self):
        """Stops the connection to the drone and closes the log"""

        # stop the connection
        self.connection.stop()

        # close the telemetry log
        self.tlog.close()

    def run(self):
        """
        Runs the connection in a while loop, same as "start" for a non-threaded connection
        """
        if self.connection.threaded:
            self.connect()
            while self.connected:
                pass
        else:
            self.start()
