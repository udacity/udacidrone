"""API definition for connections to a drone.

Defines a class to be subclassed by specific protocol implementations for 
communication with a drone.
"""

from abc import ABCMeta, abstractmethod

from fcnd_drone_api.messaging import MsgID
import traceback


class Connection():
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
        """
        self._message_listeners = {}
        self._threaded = threaded

    def on_message(self, name):
        """decorator for being able to add a listener for a specific message type

        e.g. on how you would use in in the drone class, where you have created
        a variable 'conn = Connection()'

        @conn.on_message(message_types.MSG_GLOBAL_POSITION)
        def gps_listener(_, name, gps):
            # do whatever with the gps, which will be of type GlobalPosition

        or

        @conn.on_message(MsgID.ANY)
        def all_msg_listener(_, name, msg):
            # this is a listener for all message types,
            # so break out the msg as defined by the name
        """

        def decorator(fn):
            if isinstance(name, list):
                for n in name:
                    self.add_message_listener(n, fn)
            else:
                self.add_message_listener(name, fn)

        return decorator

    def add_message_listener(self, name, fn):
        """adds a listener to the set of listeners

        adds a callback function for the specified message to the set of 
        listeners to be triggered when messages arrive.

        Args:
            name: the name of the message (see message_types.py for a valid set)
            fn: the callback function to trigger when the message comes in
        """

        if name not in self._message_listeners:
            self._message_listeners[name] = []
        if fn not in self._message_listeners[name]:
            self._message_listeners[name].append(fn)

    def remove_message_listener(self, name, fn):
        """remove a listener from the set of listeners
        
        removes a callback function for the specified message.
        
        Args:
            name: the name of the message (see message_types.py for a valid set)
            fn: the callback function to remove from the list
        """

        if name in self._message_listeners:
            if fn in self._message_listeners[name]:
                self._message_listeners[name].remove(fn)
                if len(self._message_listeners[name]) == 0:
                    del self._message_listeners[name]

    def notify_message_listeners(self, name, msg):
        """trigger all callbacks for a given message
        
        goes through list of registered listeners (callback functions) for a 
        given message and call each function in turn.
        
        Args:
            name: the name of the message (see message_types.py for a valid set)
            msg: the message data to pass to each of the listeners
        """
        for fn in self._message_listeners.get(name, []):
            try:
                print('Executing {0} callback'.format(name))
                fn(name, msg)
            except Exception as e:
                traceback.print_exc()

        for fn in self._message_listeners.get(MsgID.ANY, []):
            try:
                print('Executing {0} callback'.format(MsgID.ANY))
                fn(name, msg)
            except Exception as e:
                traceback.print_exc()

    @property
    def threaded(self):
        """bool: true if being run on a background thread """
        return self._threaded

    @abstractmethod
    def start(self):
        """command to start a connection with a drone"""
        pass

    @abstractmethod
    def stop(self):
        """command to stop a connection with a drone"""
        pass

    @abstractmethod
    def dispatch_loop(self):
        """main loop that triggers callbacks when new messages come in
        
        dispatch loop that runs and when each new message comes in over the 
        implemented protocol, translates the message to those defines in 
        `message_types.py` and triggers the callbacks.
        """
        pass

    @abstractmethod
    def arm(self):
        """command to arm the drone"""
        pass

    @abstractmethod
    def disarm(self):
        """command to disarm the drone"""
        pass

    @abstractmethod
    def take_control(self):
        """command to request control from a drone
        
        command the drone to switch into a mode that allows external control.
        e.g. for PX4 this commands 'offboard' mode, while for APM this commands 'guided' mode
        """
        pass

    @abstractmethod
    def release_control(self):
        """command to return the drone to a manual mode"""
        pass

    @abstractmethod
    def cmd_attitude(self, yaw, pitch, roll, collective):
        """command to set the desired attitude
        
        sends a command to the drone to set a specific attitude and thrust level
        
        Args:
            yaw: the desired yaw in degrees
            pitch: the desired pitch in degrees
            roll: the deisred roll in degrees
            collective: the normalized desired thrust level on [0, 1]
        """
        pass

    @abstractmethod
    def cmd_attitude_rate(self, yaw_rate, pitch_rate, roll_rate, collective):
        """command to set the desired attitude rates
        
        sends a command to the drone to set a specific attitude rate and thrust level
        
        Args:
            yaw_rate: the desired yaw rate in degrees/second
            pitch_rate: the desired pitch rate in degrees/second
            roll_rate: the desired roll rate in degrees/second
            collective: the normalized desired thrust level on [0, 1]
        """
        pass

    @abstractmethod
    def cmd_velocity(self, vn, ve, vd, heading):
        """command to set the desired velocity
        
        sends a command to the drone to set a specific velocity
        defined in a local frame (NED frame)
        
        Args:
            vn: desired north velocity component in meters/second
            ve: desired east velocity component in meters/second
            vd: desired down velocity component in meters/second (note: positive down!)
            heading: desired drone heading in degrees [0, 360)
        """
        pass

    @abstractmethod
    def cmd_motors(self, motor1, motor2, motor3, motor4):
        """command the thrust levels for each motor on a quadcopter
        
        sends a command to set the normalized thrust levels directly
        for each of the 4 motors on a quadcopter.
        
        Args:
            motor1: normalized thrust level for motor 1 on [0, 1]
            motor2: normalized thrust level for motor 2 on [0, 1]
            motor3: normalized thrust level for motor 3 on [0, 1]
            motor4: normalized thrust level for motor 4 on [0, 1]
        """
        pass

    @abstractmethod
    def cmd_position(self, n, e, d, heading):
        """command to set the desired position
        
        sends a command to the drone to set a position position
        defined in a local frame (NED frame)
        
        Args:
            n: desired north position in meters
            e: desired east position in meters
            d: desired down position in meters (note: positive down!)
            heading: desired drone heading in degrees [0, 360)
        """
        pass

    @abstractmethod
    def takeoff(self, n, e, d):
        """command to tell the drone to takeoff

        sends a takeoff command to the drone.
        note that some autopilots needs a full position for takeoff
        and since this class is not aware of current position, (n, e) 
        must be passed along with d for this command.

        Args:
            n: current north position in meters
            e: current east position in meters
            d: desired down position in meters (note: positive down!)
        """
        pass

    @abstractmethod
    def land(self, n, e):
        """command to tell the drone to land
        
        sends a land command to the drone.
        note that some autopilots needs a full position for landing
        and since this class is not aware of the current position, (n, e) 
        must be passed along (d is automatically set to 0).
        
        Args:
            n: current north position in meters
            e: current east position in meters
        """
        pass

    @abstractmethod
    def set_home_position(self, lat, lon, alt):
        """command to change the home position of the drone
        
        sends a command to change the drone's home position to that specified.
        
        Args:
            lat: desired home latitude in decimal degrees
            lon: desired home longitude in decimal degrees
            alt: desired home altitude in m AMSL
        """
        pass
