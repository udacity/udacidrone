"""API definition for connections to a drone.

Defines a class to be subclassed by specific protocol implementations for
communication with a drone.
"""

import traceback
from abc import ABCMeta, abstractmethod

from udacidrone.messaging import MsgID


class Connection(object):
    """
    Abstract class for a connection to a drone.

    Outlines the required API functions (interface) that must be
    implemented to satisfy the drone communication protocol.

    Attributes:
        __metaclass__: specifies this class as an abstract class
    """
    __metaclass__ = ABCMeta

    def __init__(self, threaded=False):
        """
        Initializes an empty dictionary of listeners for the possible different
        messages that will be sent.
        Each subclass should call this constructor in addition to any custom
        element needed for each respective communication protocol.

        Args:
            threaded (bool)
        """

        self._message_listeners = {}
        self._threaded = threaded

    def add_message_listener(self, name, fn):
        """
        Adds a callback function for the specified message to the set of
        listeners to be triggered when messages arrive.

        Args:
            name: `MsgID` name of the message
            fn: function to trigger when the message is received
        """

        if name not in self._message_listeners:
            self._message_listeners[name] = []
        if fn not in self._message_listeners[name]:
            self._message_listeners[name].append(fn)

    def remove_message_listener(self, name, fn):
        """
        Removes a callback function for the specified message if it exists.

        Args:
            name: `MsgID` name of the message
            fn: function to trigger when the message is received
        """

        if name in self._message_listeners:
            if fn in self._message_listeners[name]:
                self._message_listeners[name].remove(fn)
                if len(self._message_listeners[name]) == 0:
                    del self._message_listeners[name]

    def notify_message_listeners(self, name, msg):
        """
        Triggers all callbacks for a given message.

        Goes through list of registered listeners (callback functions) for the
        MsgID and calls each function sequentially.

        Args:
            name: `MsgID` name of the message
            msg: message data to pass to each of the listeners
        """
        for fn in self._message_listeners.get(name, []):
            try:
                fn(name, msg)
            except Exception as e:
                traceback.print_exc()

        for fn in self._message_listeners.get(MsgID.ANY, []):
            try:
                fn(name, msg)
            except Exception as e:
                traceback.print_exc()

    @property
    def threaded(self):
        """
        Returns:
            Boolean. True if the connection is running on a background thread, False otherwise.
        """
        return self._threaded

    # @abstractpropert
    @property
    @abstractmethod
    def open(self):
        """
        Returns:
            Boolean. True if connection is able to send and/or receive messages, False otherwise.
        """
        pass

    @abstractmethod
    def start(self):
        """Command to start a connection with a drone"""
        pass

    @abstractmethod
    def stop(self):
        """Command to stop a connection with a drone"""
        pass

    @abstractmethod
    def dispatch_loop(self):
        """Main loop that triggers callbacks as messages are recevied"""
        pass

    @abstractmethod
    def arm(self):
        """Command to arm the drone"""
        pass

    @abstractmethod
    def disarm(self):
        """Command to disarm the drone"""
        pass

    @abstractmethod
    def take_control(self):
        """
        Command the drone to switch into a mode that allows external control.
        e.g. for PX4 this commands 'offboard' mode, while for APM this commands 'guided' mode
        """
        pass

    @abstractmethod
    def release_control(self):
        """Command to return the drone to a manual mode"""
        pass

    @abstractmethod
    def cmd_attitude(self, roll, pitch, yaw, thrust):
        """Command to set the desired attitude and thrust

        Args:
            roll: the deisred roll in radians
            pitch: the desired pitch in radians
            yaw: the desired yaw in radians
            thrust: the normalized desired thrust level on [0, 1]
        """
        pass

    @abstractmethod
    def cmd_attitude_rate(self, roll_rate, pitch_rate, yaw_rate, thrust):
        """Command to set the desired attitude rates and thrust

        Args:
            yaw_rate: the desired yaw rate in radians/second
            pitch_rate: the desired pitch rate in radians/second
            roll_rate: the desired roll rate in radians/second
            thrust: the normalized desired thrust level on [0, 1]
        """
        pass

    @abstractmethod
    def cmd_velocity(self, vn, ve, vd, heading):
        """Command to set the desired velocity (NED frame) and heading

        Args:
            vn: desired north velocity component in meters/second
            ve: desired east velocity component in meters/second
            vd: desired down velocity component in meters/second (note: positive down!)
            heading: desired drone heading in radians
        """
        pass

    @abstractmethod
    def cmd_position(self, n, e, d, heading):
        """Command to set the desired position (NED frame) and heading

        Args:
            n: desired north position in meters
            e: desired east position in meters
            d: desired down position in meters (note: positive down!)
            heading: desired drone heading in radians
        """
        pass

    @abstractmethod
    def takeoff(self, n, e, d):
        """Command the drone to takeoff.

        Note some autopilots need a full position for takeoff
        and since this class is not aware of current position.`n` and `e`
        must be passed along with `d` for this command.

        Args:
            n: current north position in meters
            e: current east position in meters
            d: desired down position in meters (note: positive down!)
        """
        pass

    @abstractmethod
    def land(self, n, e):
        """Command the drone to land.

        Note some autopilots need a full position for landing
        and since this class is not aware of current position.`n` and `e`
        must be passed along with `d` for this command.

        Args:
            n: current north position in meters
            e: current east position in meters
        """
        pass

    @abstractmethod
    def set_home_position(self, lat, lon, alt):
        """Command to change the home position of the drone.

        Args:
            lat: desired home latitude in decimal degrees
            lon: desired home longitude in decimal degrees
            alt: desired home altitude in meters (AMSL)
        """
        pass
