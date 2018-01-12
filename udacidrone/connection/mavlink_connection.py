import os
import queue
import threading
import time
from enum import Enum

from pymavlink import mavutil

from udacidrone.messaging import MsgID

from . import message_types as mt
from . import connection

# force use of mavlink v2.0
os.environ['MAVLINK20'] = '1'
"""
Set of enums for the different possible connection types
NOTE: right now the only implemented type is PX4 mavlink
"""
CONNECTION_TYPE_MAVLINK_PX4 = 1

# CONNECTION_TYPE_MAVLINK_APM = 2
# CONNECTION_TYPE_PARROT = 3
# CONNECTION_TYPE_DJI = 4


class MainMode(Enum):
    """Constant which isn't defined in Mavlink but is useful for PX4"""
    PX4_MODE_MANUAL = 1
    PX4_MODE_OFFBOARD = 6


class PositionMask(Enum):
    """Useful masks for sending commands used in set_position_target_local_ned"""
    MASK_IGNORE_POSITION = 0x007
    MASK_IGNORE_VELOCITY = 0x038
    MASK_IGNORE_ACCELERATION = 0x1C0
    MASK_IGNORE_YAW = 0x400
    MASK_IGNORE_YAW_RATE = 0x800
    MASK_IS_FORCE = (1 << 9)
    MASK_IS_TAKEOFF = 0x1000
    MASK_IS_LAND = 0x2000
    MASK_IS_LOITER = 0x3000


class MavlinkConnection(connection.Connection):
    """
    Implementation of the required communication to a drone executed
    over the Mavlink protocol. Specifically designed with the PX4 autopilot in mind,
    and currently been tested against that autopilot software.

    Example:

        # TCP connection, protocol:ip:port
        conn = MavlinkConnection('tcp:127.0.0.1:5760')

        # Serial connection, port:baud
        conn = MavlinkConnection('5760:921600')
    """

    def __init__(self, device, threaded=False, PX4=False, send_rate=5):
        """Constructor for Mavlink based drone connection.

        Note: When threaded, the read loop runs as a daemon, meaning once all
        other processes stop the thread immediately dies, therefore some
        acitivty (e.g. a while True loop) needs to be running on the main
        thread for this thread to survive.

        Args:
            device: address to the drone, e.g. "tcp:127.0.0.1:5760" (see mavutil mavlink connection for valid options)
            threaded: bool for whether or not to run the message read loop on a separate thread
            PX4: bool for whether or not connected to a PX4 autopilot. Determines the behavior of the
                command loop (write thread)
            send_rate: the rate in Hertz (Hz) to send messages to the drone
        """

        # call the superclass constructor
        super().__init__(threaded)

        # create the connection
        if device is not "":
            self._master = mavutil.mavlink_connection(device)

        # set up any of the threading, as needed
        self._out_msg_queue = queue.Queue()  # a queue for sending data between threads
        if self._threaded:
            self._read_handle = threading.Thread(target=self.dispatch_loop)
            self._read_handle.daemon = True
        else:
            self._read_handle = None

        # need a command loop to be able to continually send certain commands
        # at the desired rate when working with PX4
        if PX4:
            self._write_handle = threading.Thread(target=self.command_loop)
            self._write_handle.daemon = True

        # management
        self._running = False
        self._target_system = 1
        self._target_component = 1

        # PX4 management
        self._using_px4 = PX4

        self._send_rate = 5

        # seconds to wait of no messages before termination
        self._timeout = 5

    @property
    def open(self):
        if self._master.port.fileno() == -1:
            return False
        return True

    def dispatch_loop(self):
        """Main loop to read from the drone.

        Continually listens to the drone connection for incoming messages.
        for each new message, parses out the mavlink, creates messages as
        defined in `message_types.py`, and triggers all callbacks registered
        for that type of message.
        Also keeps an eye on the state of connection, and if nothing has
        happened in more than 5 seconds, sends a special termination message
        to indicate that the drone connection has died.

        THIS SHOULD NOT BE CALLED DIRECTLY BY AN OUTSIDE CLASS!
        """

        last_msg_time = time.time()
        while self._running:

            # wait for a new message
            msg = self.wait_for_message()

            # if no message or a bad message was received, just move along
            if msg is None:
                continue

            print('Message received', msg)
            current_time = time.time()

            print("Time between messages", current_time - last_msg_time)

            # if we haven't heard a message in a given amount of time
            # send a termination message
            if current_time - last_msg_time > self._timeout:
                # print("timeout too long!")
                # notify listeners that the connection is closing
                self.notify_message_listeners(MsgID.CONNECTION_CLOSED, 0)

                # stop this read loop
                self._running = False

            # update the time of the last message
            last_msg_time = current_time

            # this does indeed get timestamp, should double check format
            # TODO: decide on timestamp format for messages
            timestamp = msg._timestamp
            # parse out the message based on the type and call
            # the appropriate callbacks

            # http://mavlink.org/messages/common/#GLOBAL_POSITION_INT
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                # parse out the gps position and trigger that callback
                gps = mt.GlobalFrameMessage(timestamp, float(msg.lat) / 1e7, float(msg.lon) / 1e7,
                                            float(msg.alt) / 1000)
                self.notify_message_listeners(MsgID.GLOBAL_POSITION, gps)

                # parse out the velocity and trigger that callback
                vel = mt.LocalFrameMessage(timestamp, float(msg.vx) / 100, float(msg.vy) / 100, float(msg.vx) / 100)
                self.notify_message_listeners(MsgID.LOCAL_VELOCITY, vel)

            # http://mavlink.org/messages/common/#HEARTBEAT
            elif msg.get_type() == 'HEARTBEAT':
                motors_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

                # TODO: determine if want to broadcast all current mode types
                # not just boolean on manual
                guided_mode = False

                # extract whether or not we are in offboard mode for PX4
                # (the main mode)
                main_mode = (msg.custom_mode & 0x000F0000) >> 16
                if main_mode == MainMode.PX4_MODE_OFFBOARD.value:
                    guided_mode = True

                state = mt.StateMessage(timestamp, motors_armed, guided_mode)
                self.notify_message_listeners(MsgID.STATE, state)

            # http://mavlink.org/messages/common#LOCAL_POSITION_NED
            elif msg.get_type() == 'LOCAL_POSITION_NED':
                # parse out the local positin and trigger that callback
                pos = mt.LocalFrameMessage(timestamp, msg.x, msg.y, msg.z)
                self.notify_message_listeners(MsgID.LOCAL_POSITION, pos)

                # parse out the velocity and trigger that callback
                vel = mt.LocalFrameMessage(timestamp, msg.vx, msg.vy, msg.vz)
                self.notify_message_listeners(MsgID.LOCAL_VELOCITY, vel)

            # http://mavlink.org/messages/common#HOME_POSITION
            elif msg.get_type() == 'HOME_POSITION':
                home = mt.GlobalFrameMessage(timestamp,
                                             float(msg.latitude) / 1e7,
                                             float(msg.longitude) / 1e7, float(msg.altitude) / 1000)
                self.notify_message_listeners(MsgID.GLOBAL_HOME, home)

            # http://mavlink.org/messages/common/#SCALED_IMU
            elif msg.get_type() == 'SCALED_IMU':
                # break out the message into its respective messages for here
                accel = mt.BodyFrameMessage(timestamp, msg.xacc, msg.yacc, msg.zacc)  # units are [mg]
                self.notify_message_listeners(MsgID.RAW_ACCELEROMETER, accel)

                gyro = mt.BodyFrameMessage(timestamp, msg.xgyro, msg.ygyro, msg.zgyro)  # units are [millirad/sec]
                self.notify_message_listeners(MsgID.RAW_GYROSCOPE, gyro)

            # http://mavlink.org/messages/common#SCALED_PRESSURE
            elif msg.get_type() == 'SCALED_PRESSURE':
                pressure = mt.BodyFrameMessage(timestamp, 0, 0, msg.press_abs)  # unit is [hectopascal]
                self.notify_message_listeners(MsgID.BAROMETER, pressure)

            # http://mavlink.org/messages/common#DISTANCE_SENSOR
            elif msg.get_type() == 'DISTANCE_SENSOR':
                direction = 0
                # TODO: parse orientation
                # orientation = msg.orientation
                meas = mt.DistanceSensorMessage(timestamp,
                                                float(msg.min_distance) / 100,
                                                float(msg.max_distance) / 100, direction,
                                                float(msg.current_distance) / 100, float(msg.covariance) / 100)
                self.notify_message_listeners(MsgID.DISTANCE_SENSOR, meas)

            # http://mavlink.org/messages/common#ATTITUDE_QUATERNION
            elif msg.get_type() == 'ATTITUDE_QUATERNION':
                timestamp = msg.time_boot_ms
                # TODO: check if mask notifies us to ignore a field

                fm = mt.FrameMessage(timestamp, msg.q1, msg.q2, msg.q3, msg.q4)
                self.notify_message_listeners(MsgID.ATTITUDE, fm)

                gyro = mt.BodyFrameMessage(timestamp, msg.rollspeed, msg.pitchspeed, msg.yawspeed)
                self.notify_message_listeners(MsgID.RAW_GYROSCOPE, gyro)

            # DEBUG
            elif msg.get_type() == 'STATUSTEXT':
                # print("[autopilot message] " + msg.text.decode("utf-8"))
                pass

    def command_loop(self):
        """
        Main loop for sending commands.

        Loop that is run a separate thread to be able to send messages to the
        target drone.  Uses the message queue `self._out_msg_queue` as the
        queue of messages to run.
        """

        # default to sending a position command to (0,0,0)
        # this needs to be sending commands at a rate of at lease 2Hz in order
        # for PX4 to allow a switch into offboard control.
        mask = (PositionMask.MASK_IGNORE_YAW_RATE.value | PositionMask.MASK_IGNORE_ACCELERATION.value |
                PositionMask.MASK_IGNORE_VELOCITY.value)
        high_rate_command = self._master.mav.set_position_target_local_ned_encode(
            0, self._target_system, self._target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0)

        last_write_time = time.time()
        while self._running:

            # rate limit the loop
            current_time = time.time()
            if (current_time - last_write_time) < 1.0 / self._send_rate:
                continue
            last_write_time = time.time()

            # empty out the queue of pending messages
            # NOTE: Queue class is synchronized and is thread safe already!
            msg = None
            while not self._out_msg_queue.empty():
                try:
                    msg = self._out_msg_queue.get_nowait()
                except queue.Empty:
                    # if there is no msgs in the queue, will just continue
                    pass
                else:
                    # either set this is as the high rate command
                    # to repeatedly send or send it immediately
                    if msg.get_type() == 'SET_POSITION_TARGET_LOCAL_NED' or msg.get_type() == 'SET_ATTITUDE_TARGET':
                        high_rate_command = msg
                    else:
                        self._master.mav.send(msg)
                    self._out_msg_queue.task_done()

            # continually want to send the high rate command
            self._master.mav.send(high_rate_command)

    def wait_for_message(self):
        """
        Wait for a new mavlink message calls pymavlink's blocking read function to read
        a next message, blocking for up to a timeout of 1s.

        Returns:
            Mavlink message that was read or `None` if the message was invalid.
        """

        # NOTE: this returns a mavlink message
        # this function should not be called outside of this class!
        msg = self._master.recv_match(blocking=True, timeout=1)
        if msg is None:
            # no message received
            return None
        else:
            if (msg.get_type() == 'BAD_DATA'):
                # no message that is useful
                return None

            # send a heartbeat message back, since this needs to be
            # constantly sent so the autopilot knows this exists
            if msg.get_type() == 'HEARTBEAT':
                # send -> type, autopilot, base mode, custom mode, system status
                outmsg = self._master.mav.heartbeat_encode(mavutil.mavlink.MAV_TYPE_GCS,
                                                           mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                                           mavutil.mavlink.MAV_STATE_ACTIVE)
                self.send_message(outmsg)

            # pass the message along to be handled by this class
            return msg

    def start(self):
        # start the main thread
        self._running = True

        # start the command loop
        # this is only needed when working with PX4 and we need to enforce
        # a certain rate of messages being sent
        if self._using_px4:
            self._write_handle.start()

        # start the dispatch loop, either threaded or not
        if self._threaded:
            self._read_handle.start()
        else:
            # NOTE: this is a full blocking function here!!
            self.dispatch_loop()

    def stop(self):
        # stop the dispatch and command while loops
        self._running = False

        # NOTE: no need to call join on the threads
        # as both threads are daemon threads

        # close the connection
        print("Closing connection ...")
        self._master.close()

    def send_message(self, msg):
        """Send a given mavlink message to the drone. If connected with a PX4 autopilot,
        add the MAVLinkMessage to the command queue to be handled by the command loop
        (running in the write thread).  Otherwise immediately send the message.

        :param msg: MAVLinkMessage to be sent to the drone
        """

        # if we are using PX4, means we are also using out command loop
        # in a separate thread and therefore need to send the data to that
        # thread using the queue
        #
        # if we are not using PX4, then just immediately send the message
        if self._using_px4:
            # NOTE: queue is already thread safe
            # no need to handle additional locks
            self._out_msg_queue.put(msg)
        else:
            self._master.mav.send(msg)

    def send_long_command(self, command_type, param1, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
        """
        Packs and sends a Mavlink COMMAND_LONG message

        Args:
            command_type: the command type, as defined by MAV_CMD_*
            param1: param1 as defined by the specific command
            param2: param2 as defined by the specific command (default: {0})
            param3: param3 as defined by the specific command (default: {0})
            param4: param4 as defined by the specific command (default: {0})
            param5: param5 (x) as defined by the specific command (default: {0})
            param6: param6 (y) as defined by the specific command (default: {0})
            param7: param7 (z) as defined by the specific command (default: {0})
        """
        confirmation = 0  # may want this as an input.... used for repeat messages
        msg = self._master.mav.command_long_encode(self._target_system, self._target_component, command_type,
                                                   confirmation, param1, param2, param3, param4, param5, param6, param7)
        self.send_message(msg)

    def arm(self):
        # send an arm command through mavlink
        self.send_long_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)

    def disarm(self):
        # send a disarm command
        self.send_long_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)

    def take_control(self):
        mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  # tells system to use PX4 custom commands
        custom_mode = MainMode.PX4_MODE_OFFBOARD.value
        custom_sub_mode = 0  # not used for manual/offboard
        self.send_long_command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mode, custom_mode, custom_sub_mode)

    def release_control(self):
        mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  # tells system to use PX4 custom commands
        custom_mode = MainMode.PX4_MODE_MANUAL.value
        custom_sub_mode = 0  # not used for manual/offboard
        self.send_long_command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mode, custom_mode, custom_sub_mode)

    def cmd_attitude(self, roll, pitch, yawrate, thrust):
        time_boot_ms = 0  # this does not need to be set to a specific time
        # TODO: convert the attitude to a quaternion
        frame_msg = mt.FrameMessage(0.0, roll, pitch, 0.0)
        q = [frame_msg.q0, frame_msg.q1, frame_msg.q2, frame_msg.q3]
        mask = 0b00000011
        msg = self._master.mav.set_attitude_target_encode(time_boot_ms, self._target_system, self._target_component,
                                                          mask, q, 0, 0, yawrate, thrust)
        self.send_message(msg)

    def cmd_attitude_rate(self, roll_rate, pitch_rate, yaw_rate, thrust):
        time_boot_ms = 0  # this does not need to be set to a specific time
        q = [0.0, 0.0, 0.0, 0.0]
        mask = 0b10000000
        msg = self._master.mav.set_attitude_target_encode(time_boot_ms, self._target_system, self._target_component,
                                                          mask, q, roll_rate, pitch_rate, yaw_rate, thrust)
        self.send_message(msg)

    def cmd_velocity(self, vn, ve, vd, heading):
        time_boot_ms = 0  # this does not need to be set to a specific time
        mask = (PositionMask.MASK_IGNORE_YAW_RATE.value | PositionMask.MASK_IGNORE_ACCELERATION.value |
                PositionMask.MASK_IGNORE_POSITION.value)
        msg = self._master.mav.set_position_target_local_ned_encode(
            time_boot_ms, self._target_system, self._target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, 0, 0,
            0, vn, ve, vd, 0, 0, 0, heading, 0)
        self.send_message(msg)

    def cmd_motors(self, motor1, motor2, motor3, motor4):
        # TODO: implement this
        pass

    def cmd_position(self, n, e, d, heading):
        time_boot_ms = 0  # this does not need to be set to a specific time
        # mask = PositionMask.MASK_IS_LOITER.value
        mask = (PositionMask.MASK_IGNORE_YAW_RATE.value | PositionMask.MASK_IGNORE_ACCELERATION.value |
                PositionMask.MASK_IGNORE_VELOCITY.value)
        msg = self._master.mav.set_position_target_local_ned_encode(
            time_boot_ms, self._target_system, self._target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, n, e,
            d, 0, 0, 0, 0, 0, 0, heading, 0)
        self.send_message(msg)

    def takeoff(self, n, e, d):
        # for mavlink to PX4 need to specify the NED location for landing
        # since connection doesn't keep track of this info, have drone send it
        # abstract away that part in the drone class
        time_boot_ms = 0  # this does not need to be set to a specific time
        mask = PositionMask.MASK_IS_TAKEOFF.value
        mask |= (PositionMask.MASK_IGNORE_YAW_RATE.value | PositionMask.MASK_IGNORE_YAW.value |
                 PositionMask.MASK_IGNORE_ACCELERATION.value | PositionMask.MASK_IGNORE_VELOCITY.value)
        msg = self._master.mav.set_position_target_local_ned_encode(
            time_boot_ms, self._target_system, self._target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, n, e,
            d, 0, 0, 0, 0, 0, 0, 0, 0)
        self.send_message(msg)

    def land(self, n, e):
        # for mavlink to PX4 need to specify the NED location for landing
        # since connection doesn't keep track of this info, have drone send it
        # abstract away that part in the drone class
        d = 0  # going to land, so just set d to 0
        time_boot_ms = 0  # this does not need to be set to a specific time
        mask = PositionMask.MASK_IS_LAND.value
        mask |= (PositionMask.MASK_IGNORE_YAW_RATE.value | PositionMask.MASK_IGNORE_YAW.value |
                 PositionMask.MASK_IGNORE_ACCELERATION.value | PositionMask.MASK_IGNORE_VELOCITY.value)
        msg = self._master.mav.set_position_target_local_ned_encode(
            time_boot_ms, self._target_system, self._target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, n, e,
            d, 0, 0, 0, 0, 0, 0, 0, 0)
        self.send_message(msg)

    def set_home_position(self, lat, lon, alt):
        self.send_long_command(mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, lat, lon, alt)
