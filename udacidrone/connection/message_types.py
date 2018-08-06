"""
TODO: This likely to undergo some changes.

Message Types

custom set of message types to use between a specific connection type and
the drone class.
this enables abstracting away the protocol specific messages so different
protocols can be used with the same student facing interface and code.

NOTE: besides the state message, all messages are in the frame they are
defined in.

NOTE: to ensure minimal errors due to typos, use the MSG_* constants when
registering for specific messages

Attributes:
    MSG_ALL: flag to be used to register a listener for all messages
    MSG_STATE: name of the state message
    MSG_GLOBAL_POSITION: name of the global position message [StateMessage]
    MSG_LOCAL_POSITION: name of the local position message [LocalFrameMessage]
    MSG_GLOBAL_HOME: name of the global home message [GlobalFrameMessage]
    MSG_VELOCITY: name of the velocity message [LocalFrameMessage]
    MSG_CONNECTION_CLOSED: name of the message sent when the connection is closed (no data)
    MSG_RAW_GYROSCOPE: name of the raw gyro message [BodyFrameMessage]
    MSG_RAW_ACCELEROMETER: name of the raw acceleromater message [BodyFrameMessage]
    MSG_BAROMETER: name of the barometer message [LocalFrameMessage - only down populated]
    MSG_ATTITUDE: name of attitude message [FrameMessage]
"""
import numpy as np


class Message:
    """Message super class

    Attributes:
        _time: the message time
    """

    def __init__(self, time):
        self._time = time

    @property
    def time(self):
        """ float: message time in [ms] """
        return self._time


class StateMessage(Message):
    """State information message

    message to carry drone state information

    Attributes:
        _armed: whether or not drone is armed
        _guided: whether or not drone can be commanded from python
    """

    def __init__(self, time, armed, guided, status=0):
        super().__init__(time)
        self._armed = armed
        self._guided = guided
        self._status = status

    @property
    def armed(self):
        """bool: true if the drone is armed and ready to fly """
        return self._armed

    @property
    def guided(self):
        """bool: true if the drone can be commanded from python """
        return self._guided

    @property
    def status(self):
        """int: status value from the autopilot not corresponding to anything in particular """
        return self._status


class GlobalFrameMessage(Message):
    """Global frame message

    message to carry information in a global frame

    Attributes:
        _latitude: latitude in degrees
        _longitude: longitude in degrees
        _altitude: altitude in meters above mean sea level (AMSL)
    """

    def __init__(self, time, latitude, longitude, altitude):
        super().__init__(time)
        self._longitude = longitude
        self._latitude = latitude
        self._altitude = altitude

    @property
    def longitude(self):
        """float: longitude in degrees """
        return self._longitude

    @property
    def latitude(self):
        """float: latitude in degrees """
        return self._latitude

    @property
    def altitude(self):
        """float: altitude in meters above sea level """
        return self._altitude

    @property
    def global_vector(self):
        """float array: numpy array of [longitude, latitude, altitude] """
        return np.array([self._longitude, self._latitude, self._altitude])


class LocalFrameMessage(Message):
    """Local frame message

    message to carry information in a local (NED) frame

    Attributes:
        _north: north position in meters
        _east: east position in meters
        _down: down position in meters (above takeoff point, positive down)
    """

    def __init__(self, time, north, east, down):
        super().__init__(time)
        self._north = north
        self._east = east
        self._down = down

    @property
    def north(self):
        """float: north position in meters """
        return self._north

    @property
    def east(self):
        """float: east position in meters """
        return self._east

    @property
    def down(self):
        """float: down position in meters """
        return self._down

    @property
    def local_vector(self):
        """float array: numpy array of NED position, [north, east, down] """
        return np.array([self._north, self._east, self._down])


class BodyFrameMessage(Message):
    """Body frame message

    message to carry information in a body frame

    Attributes:
        _x: x value
        _y: y value
        _z: z value
    """

    def __init__(self, time, x, y, z):
        super().__init__(time)
        self._x = x
        self._y = y
        self._z = z

    @property
    def x(self):
        """float: x value """
        return self._x

    @property
    def y(self):
        """float: y value """
        return self._y

    @property
    def z(self):
        """float: z value """
        return self._z

    @property
    def body_vector(self):
        """float array: numpy array of the values, [x, y, z] """
        return np.array([self._x, self._y, self._z])


class FrameMessage(Message):
    """Message representating frame information

    Messages defining the rotation between frames (Euler angles or Quaternions)

    Attributes:
        _roll: drone roll in radians
        _pitch: drone pitch in radians
        _yaw: drone yaw in radians
        _q0: 0th element of quaterion
        _q1: 1th element of quaterion
        _q2: 2th element of quaterion
        _q3: 3th element of quaterion
    """

    def __init__(self, *args):
        if len(args) == 4:
            self.init_euler(args[0], args[1], args[2], args[3])
        elif len(args) == 5:
            self.init_quaternion(args[0], args[1], args[2], args[3], args[4])

    def init_euler(self, time, roll, pitch, yaw):
        super().__init__(time)

        self._roll = roll
        self._pitch = pitch
        self._yaw = yaw

        sp = np.sin(pitch / 2.0)
        cp = np.cos(pitch / 2.0)
        sr = np.sin(roll / 2.0)
        cr = np.cos(roll / 2.0)
        sy = np.sin(yaw / 2.0)
        cy = np.cos(yaw / 2.0)

        self._q0 = cr * cp * cy + sr * sp * sy
        self._q1 = sr * cp * cy - cr * sp * sy
        self._q2 = cr * sp * cy + sr * cp * sy
        self._q3 = cr * cp * sy - sr * sp * cy

    def init_quaternion(self, time, q0, q1, q2, q3):
        super().__init__(time)
        self._q0 = q0
        self._q1 = q1
        self._q2 = q2
        self._q3 = q3

        self._roll = np.arctan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1**2 + q2**2))
        self._pitch = np.arcsin(2.0 * (q0 * q2 - q3 * q1))
        self._yaw = np.arctan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2**2 + q3**2))

    @property
    def roll(self):
        """roll in radians"""
        return self._roll

    @property
    def pitch(self):
        """pitch in radians"""
        return self._pitch

    @property
    def yaw(self):
        """yaw in radians"""
        return self._yaw

    @property
    def euler_angles(self):
        """float array: numpy array of the euler angles [roll, pitch, yaw] """
        return np.array([self._roll, self._pitch, self._yaw])

    @property
    def quaternions(self):
        """float array: numpy array of the quaternion vector """
        return np.array([self._q0, self._q1, self._q2, self._q3])

    @property
    def q0(self):
        """float: 0th element of quaternion """
        return self._q0

    @property
    def q1(self):
        """float: 1st element of quaternion """
        return self._q1

    @property
    def q2(self):
        """float: 2nd element of quaternion """
        return self._q2

    @property
    def q3(self):
        """float: 3rd element of quaternion """
        return self._q3


class DistanceSensorMessage(Message):
    """Message for distance sensor (e.g. Lidar) information

    the properties of and measurement from a given distance sensor onboard
    the drone.

    Attributes:
        _min_distance: minimum detectable distance in meters
        _max_distance: maximum detectable distance in meters
        _direction: the heading of the sensor for this measurement in radians
        _measurement: the distance measured in meters
        _covariance: the covariance of the measurement
    """

    def __init__(self, time, min_distance, max_distance, direction, measurement, covariance):
        super().__init__(time)
        self._min_distance = min_distance
        self._max_distance = max_distance
        self._direction = direction
        self._measurement = measurement
        self._covariance = covariance

    @property
    def measurement(self):
        """
        (float, float, float): tuple containing the measurement information defined as
        (direction, distance, covariance)
        """
        return (self._direction, self._measurement, self._covariance)

    @property
    def properties(self):
        """ (float, float): tuple containing the properties of the sensor defined as (min distance, max distance) """
        return (self._min_distance, self._max_distance)
