import numpy as np

from fcnd_drone_api.logging import Logger
from fcnd_drone_api.messaging import MsgID


class Drone:

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

        # If there is an active connection to a simulated or physical drone.
        self._connected = False

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
            MsgID.ATTITUDE_TARGET: self._update_euler_angle,
            MsgID.BAROMETER: self._update_barometer
        }

        # self.conn.add_message_listener('*',self.on_message_receive)

        self._message_listeners = {}
        self.callbacks()
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
        return self._connected

    def _update_state(self, msg):
        self._armed = msg.armed
        self._guided = msg.guided
        self._connected = True

    @property
    def euler_angle(self):
        """Roll, pitch, yaw euler angles in radians"""
        return np.array([self._roll, self._pitch, self._yaw])

    def _update_euler_angle(self, msg):
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
        return np.array([self._gryo_x, self._gyro_y, self._gyro_z])

    def _update_gyro_raw(self, msg):
        self._gyro_x = msg.x
        self._gyro_y = msg.y
        self._gyro_z = msg.z

    @property
    def barometer(self):
        return np.array(self._baro_altitude)

    def _update_barometer(self, msg):
        self._baro_altitude = msg.altitude

    def callbacks(self):

        @self.connection.on_message(MsgID.ANY)
        def on_message_receive(_, msg_name, msg):
            print('message received', msg_name)
            if msg_name == MsgID.CONNECTION_CLOSED:
                self.stop()
            """Sorts incoming messages, updates the drone state variables and runs callbacks"""
            if msg_name in self._update_property.keys():
                self._update_property[msg_name](msg)

            self.notify_message_listeners(msg_name, msg)

            self.log_telemetry(msg_name, msg)

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

    def msg_callback(self, name):
        """decorator for being able to add a listener for a specific message type

        @self.msg_callback(message_types.MSG_GLOBAL_POSITION)
        def gps_listener(name, gps):
            # do whatever with the gps, which will be of type GlobalPosition

        or

        @self.msg_callback('*')
        def all_msg_listener(name, msg):
            # this is a listener for all message types, so break out the msg as defined by the name

        These listeners need to be defined within the method self.callbacks() or directly within
        self.__init__() which calls self.callbacks.

        Callbacks defined with decorators cannot be removed, use add_message_listener/remove_message_listener
        if the callback needs to be removed.
        """

        def decorator(fn):
            if isinstance(name, list):
                for n in name:
                    self.add_message_listener(n, fn)
            else:
                self.add_message_listener(name, fn)

        return decorator

    def add_message_listener(self, name, fn):
        """Add the function, fn, as a callback for the message type, name

        For example:
            self.add_message_listener(message_types.MSG_GLOBAL_POSITION,global_msg_listener)
            OR
            self.add_message_listener('*',all_msg_listener)

        These can be added anywhere in the code and are identical to initializing a callback with the decorator
        """
        if name not in self._message_listeners:
            self._message_listeners[name] = []
        if fn not in self._message_listeners[name]:
            self._message_listeners[name].append(fn)
        print('after', self._message_listeners)

    def remove_message_listener(self, name, fn):
        """Remove the function, fn, as a callback for the message type, name

        For example:
            self.remove_message_listener(message_types.MSG_GLOBAL_POSITION,global_msg_listener)

        """
        name = str(name)
        if name in self._message_listeners:
            if fn in self._message_listeners[name]:
                self._message_listeners[name].remove(fn)
                if len(self._message_listeners[name]) == 0:
                    del self._message_listeners[name]

    def notify_message_listeners(self, name, msg):
        """Passes the message to the appropriate listeners"""
        for fn in self._message_listeners.get(name, []):
            try:
                # fn(self, name, msg)
                fn(name, msg)
            except Exception as e:
                print('>>> Exception in message handler for %s' % name)
                print('>>> ' + str(e))

        for fn in self._message_listeners.get(MsgID.ANY, []):
            try:
                # fn(self, name, msg)
                fn(name, msg)
            except Exception as e:
                print('>>> Exception in message handler for %s' % name)
                print('>>> ' + str(e))

    #
    # Command method wrappers
    #

    def arm(self):
        """Send an arm command to the drone"""
        try:
            self.connection.arm()
        except:
            print("arm command not defined")

    def disarm(self):
        """Send a disarm command to the drone"""
        try:
            self.connection.disarm()
        except:
            print("disarm command not defined")

    def take_control(self):
        """If the drone is in guided mode this will switch to manual mode"""
        print('Take Control Messsage')
        try:
            self.connection.take_control()
        except:
            print("take_control command not defined")

    def release_control(self):
        """Take control of the drone """
        try:
            self.connection.release_control()
        except:
            print("release_control command not defined")

    def cmd_position(self, north, east, down, heading):
        """ Command the local position and drone heading
            north: local north in meters
            east: local east in meters
            down: local down in meters (positive down)
            heading: drone yaw in degrees
        """
        try:
            self.connection.cmd_position(north, east, down, heading)
        except:
            print("cmd_position not defined")

    def takeoff(self, target_altitude):
        """Command the drone to takeoff to the target_alt (in meters)"""
        try:
            self.connection.takeoff(self.local_position[0], self.local_position[1], target_altitude)
        except:
            print("takeoff no defined")

    def land(self):
        """Command the drone to land at its current position"""
        try:
            self.connection.land(self.local_position[0], self.local_position[1])
        except:
            print("land not defined")

    def cmd_attitude_rate(self, roll_rate, pitch_rate, yaw_rate, collective):
        """Command the drone orientation rates
            roll_rate,pitch_rate,yaw_rate: in deg/s
            collective: upward acceleration in m/s**2
        """
        try:
            self.connection.cmd_attitude_rate(roll_rate, pitch_rate, yaw_rate, collective)
        except:
            print("cmd_attitude_rate not defined")

    def cmd_velocity(self, velocity_north, velocity_east, velocity_down, heading):
        """Command the drone velocity
            north_velocity,east_velocity,down_velocity: in m/s
            heading: in degrees
        """
        try:
            self.connection.cmd_velocity(velocity_north, velocity_east, velocity_down, heading)
        except:
            print("cmd_velocity not defined")

    def cmd_motors(self, motor_rpm):
        """Command the rmp of the motors"""
        try:
            self.connection.cmd_motors(motor_rpm)
        except:
            print("cmd_motors not defined")

    def set_home_position(self, longitude, latitude, altitude):
        """Set the drone's home position to these coordinates"""
        try:
            self.connection.set_home_position(latitude, longitude, altitude)
        except:
            print("set_home_position not defined")

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
        self._connected = False

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
