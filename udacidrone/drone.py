import time
import traceback

import numpy as np

from udacidrone.logging import Logger
from udacidrone.messaging import MsgID


class Drone(object):
    """
    Drone class
    """

    def __init__(self, connection, tlog_directory="Logs", tlog_name="TLog.txt"):

        self.connection = connection

        self._message_time = 0.0
        self._message_frequency = 0.0
        self._time_bias = 0.0
        # Global position in degrees (int)
        # Altitude is in meters
        self._longitude = 0
        self._latitude = 0
        self._altitude = 0
        self._global_position_time = 0.0
        self._global_position_frequency = 0.0

        # Reference home position in degrees (int)
        # Altitude is in meters
        self._home_longitude = 0
        self._home_latitude = 0
        self._home_altitude = 0
        self._home_position_time = 0.0
        self._home_position_frequency = 0.0

        # Local positions in meters from the global home (float)
        # In NED frame
        self._north = 0.0
        self._east = 0.0
        self._down = 0.0
        self._local_position_time = 0.0
        self._local_position_frequency = 0.0

        # Locally oriented velocity in meters/second
        # In NED frame
        self._velocity_north = 0.0
        self._velocity_east = 0.0
        self._velocity_down = 0.0
        self._local_velocity_time = 0.0
        self._local_velocity_frequency = 0.0

        # If the drone is armed the motors are powered and the rotors are spinning.
        self._armed = False

        # If the drone is guided it is being autonomously controlled,
        # the other opposite would be manual control.
        self._guided = False
        
        # An integer to pass along random status changes specific for different vehicles
        self._status = 0
        self._state_time = 0.0
        self._state_frequency = 0.0

        # Euler angles in radians
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._attitude_time = 0.0
        self._attitude_frequency = 0.0

        # Drone body accelerations
        self._acceleration_x = 0.0
        self._acceleration_y = 0.0
        self._acceleration_z = 0.0
        self._acceleration_time = 0.0
        self._acceleration_frequency = 0.0

        # Drone gyro rates or angular velocities in radians/second
        # TODO: Explain what x, y, and z are.
        self._gyro_x = 0.0
        self._gyro_y = 0.0
        self._gyro_z = 0.0
        self._gyro_time = 0.0
        self._gyro_frequency = 0.0

        # Barometer
        # TODO: better explanation
        self._baro_altitude = 0.0
        self._baro_time = 0.0
        self._baro_frequency = 0.0

        # Initializing telemetry logger
        self.tlog = Logger(tlog_directory, tlog_name)

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
            if (((msg.time - self._message_time) > 0.0)):
                self._message_frequency = 1.0 / (msg.time - self._message_time)
                self._message_time = msg.time
                self._time_bias = msg.time - time.time()

            if msg_name == MsgID.CONNECTION_CLOSED:
                self.stop()
            if msg_name in self._update_property.keys():
                self._update_property[msg_name](msg)
            self.notify_callbacks(msg_name, msg)  # pass it along to these listeners
            self.log_telemetry(msg_name, msg)

        # add the above callback function as a listener for all connection messages
        self.connection.add_message_listener(MsgID.ANY, on_message_receive)

    @property
    def global_position(self):
        return np.array([self._longitude, self._latitude, self._altitude])

    @property
    def global_position_time(self):
        return self._global_position_time

    def _update_global_position(self, msg):
        self._longitude = msg.longitude
        self._latitude = msg.latitude
        self._altitude = msg.altitude
        if (msg.time - self._global_position_time) > 0.0:
            self._global_position_frequency = 1.0 / (msg.time - self._global_position_time)

        self._global_position_time = msg.time

    @property
    def global_home(self):
        return np.array([self._home_longitude, self._home_latitude, self._home_altitude])

    @property
    def home_position_time(self):
        return self._home_position_time

    def _update_global_home(self, msg):
        self._home_longitude = msg.longitude
        self._home_latitude = msg.latitude
        self._home_altitude = msg.altitude
        if (msg.time - self._home_position_time) < 0.0:
            self._home_position_frequency = 1.0 / (msg.time - self._home_position_time)
        self._home_position_time = msg.time

    @property
    def local_position(self):
        return np.array([self._north, self._east, self._down])

    @property
    def local_position_time(self):
        return self._local_position_time

    def _update_local_position(self, msg):
        self._north = msg.north
        self._east = msg.east
        self._down = msg.down
        if (msg.time - self._local_position_time) > 0.0:
            self._local_position_frequency = 1.0 / (msg.time - self._local_position_time)
        self._local_position_time = msg.time

    @property
    def local_velocity(self):
        return np.array([self._velocity_north, self._velocity_east, self._velocity_down])

    @property
    def local_velocity_time(self):
        return self._local_velocity_time

    def _update_local_velocity(self, msg):
        self._velocity_north = msg.north
        self._velocity_east = msg.east
        self._velocity_down = msg.down
        if (msg.time - self._local_velocity_time) > 0.0:
            self._local_velocity_frequency = 1.0 / (msg.time - self._local_velocity_time)
        self._local_velocity_time = msg.time

    @property
    def armed(self):
        return self._armed

    @property
    def guided(self):
        return self._guided

    @property
    def connected(self):
        return self.connection.open

    @property
    def state_time(self):
        return self._state_time
    
    @property
    def status(self):
        return self._status

    def _update_state(self, msg):
        self._armed = msg.armed
        self._guided = msg.guided
        if (msg.time - self._state_time) > 0.0:
            self._state_frequency = 1.0 / (msg.time - self._state_time)
        self._state_time = msg.time
        self._status = msg.status

    @property
    def attitude(self):
        """Roll, pitch, yaw euler angles in radians"""
        return np.array([self._roll, self._pitch, self._yaw])

    @property
    def attitude_time(self):
        return self._attitude_time

    def _update_attitude(self, msg):
        self._roll = msg.roll
        self._pitch = msg.pitch
        self._yaw = msg.yaw
        if (msg.time - self._attitude_time) > 0.0:
            self._attitude_frequency = 1.0 / (msg.time - self._attitude_time)
            self._attitude_time = msg.time

    @property
    def acceleration_raw(self):
        return np.array([self._acceleration_x, self._acceleration_y, self._acceleration_z])

    @property
    def acceleration_time(self):
        return self._acceleration_time

    def _update_acceleration_raw(self, msg):
        self._acceleration_x = msg.x
        self._acceleration_y = msg.y
        self._acceleration_z = msg.z
        if (msg.time - self._acceleration_time) > 0.0:
            self._acceleration_frequency = 1.0 / (msg.time - self._acceleration_time)
        self._acceleration_time = msg.time

    @property
    def gyro_raw(self):
        """Angular velocites in radians/second"""
        return np.array([self._gyro_x, self._gyro_y, self._gyro_z])

    @property
    def gyro_time(self):
        return self._gyro_time

    def _update_gyro_raw(self, msg):
        self._gyro_x = msg.x
        self._gyro_y = msg.y
        self._gyro_z = msg.z
        if (msg.time - self._gyro_time) > 0.0:
            self._gyro_frequency = 1.0 / (msg.time - self._gyro_time)
        self._gyro_time = msg.time

    @property
    def barometer(self):
        return np.array(self._baro_altitude)

    @property
    def barometer_time(self):
        return self._baro_time

    def _update_barometer(self, msg):
        self._baro_altitude = msg.z
        self._baro_frequency = 1.0 / (msg.time - self._baro_time)
        self._baro_time = msg.time

    def log_telemetry(self, msg_name, msg):
        """Save the msg information to the telemetry log"""
        if self.tlog.open:
            self.tlog.log_telemetry_msg(msg_name, msg)
            # data = [msg_name]
            # data.append(msg.time)
            # for k in msg.__dict__.keys():
            #     if k != '_time':
            #         data.append(msg.__dict__[k])
            # self.tlog.log_telemetry_data(data)

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
                # print('Drone executing {0} callback'.format(MsgID.ANY))
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
        """Send a command to the drone to switch to guided (autonomous) mode.

        Essentially control the drone with code.
        """
        try:
            self.connection.take_control()
        except Exception as e:
            traceback.print_exc()

    def release_control(self):
        """Send a command to the drone to switch to manual mode.

        Essentially you control the drone manually via some interface."""
        try:
            self.connection.release_control()
        except Exception as e:
            traceback.print_exc()

    def cmd_position(self, north, east, altitude, heading):
        """Command the local position and drone heading.

        Args:
            north: local north in meters
            east: local east in meters
            altitude: altitude above ground in meters
            heading: drone yaw in radians
        """
        try:
            # connection cmd_position is defined as NED, so need to flip the sign
            # on altitude
            self.connection.cmd_position(north, east, -altitude, heading)
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

    def cmd_attitude(self, roll, pitch, yaw, thrust):
        """Command the drone through attitude command

        Args:
            roll: in radians
            pitch: in randians
            yaw_rate: in radians
            thrust: normalized thrust on [0, 1] (0 being no thrust, 1 being full thrust)
        """
        try:
            self.connection.cmd_attitude(roll, pitch, yaw, thrust)
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

    def set_home_position(self, longitude, latitude, altitude):
        """Set the drone's home position to these coordinates"""
        try:
            self.connection.set_home_position(latitude, longitude, altitude)
        except Exception as e:
            traceback.print_exc()

    def set_home_as_current_position(self):
        """Set the drone's home position to its current position"""
        self.set_home_position(self._longitude, self._latitude, self._altitude)

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
