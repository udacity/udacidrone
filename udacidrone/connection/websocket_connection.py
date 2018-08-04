import asyncio
import logging
import os
import platform
import time
from io import BytesIO

import websockets
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from udacidrone.messaging import MsgID

from . import connection
from . import message_types as mt
from .mavlink_utils import MainMode, PositionMask, dispatch_message

# force use of mavlink v2.0
os.environ['MAVLINK20'] = '1'

if platform.system() is not 'Windows':
    import uvloop
    asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())
logging.getLogger('asyncio').setLevel(logging.WARNING)

# logger = logging.getLogger('udacidrone')


class WebSocketConnection(connection.Connection):
    """
    Implementation of the required communication to a drone executed
    over the WebSocket protocol. Messages sent and received are encoded/decoded
    by MAVLink.

    The main use of this connection is to allow communication with the FCND simulator
    in the classroom (while in the browser) since current limitations do not allow for communication
    via TCP/UDP directly.

    Example:

        conn = WebSocketConnection('ws://127.0.0.1:5760')

    """

    def __init__(self, uri, timeout=5):
        """
        Args:
            uri: address of the websocket server, e.g. "ws://127.0.0.1:5760"
            timeout: the time limit in seconds to wait for a message prior to closing connection
        """

        # call the superclass constructor
        super().__init__(threaded=False)

        self._uri = uri
        self._f = BytesIO()
        self._mav = mavlink.MAVLink(self._f)
        # This will be set with self._uri when the async event loop is
        # started
        self._ws = None
        self._q = asyncio.Queue()

        # management
        self._target_system = 1
        self._target_component = 1

        self._running = False

        # seconds to wait of no messages before termination
        self._timeout = timeout

    @property
    def uri(self):
        return self._uri

    @property
    def timeout(self):
        return self._timeout

    @property
    def open(self):
        if self._ws is None:
            return False
        return self._ws.open

    def decode_message(self, msg):
        """
        Decodes Mavlink message.
        """
        return self._mav.decode(bytearray(msg))

    async def _dispatch_loop(self):
        """
        Continually listens to the drone connection for incoming messages.
        For each new message, decodes the MAVLink, creates messages as
        defined in `message_types.py`, and triggers all callbacks registered
        for that type of message.
        """
        last_msg_time = time.time()
        async with websockets.connect(self._uri) as ws:
            self._ws = ws
            while self._running:
                msg = await ws.recv()
                msg = self.decode_message(msg)

                if msg is None or msg.get_type() == 'BAD_DATA':
                    continue

                # send a heartbeat message back, since this needs to be
                # constantly sent so the autopilot knows this exists
                if msg.get_type() == 'HEARTBEAT':
                    # send -> type, autopilot, base mode, custom mode, system status
                    outmsg = self._mav.heartbeat_encode(mavutil.mavlink.MAV_TYPE_GCS,
                                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                                        mavutil.mavlink.MAV_STATE_ACTIVE)
                    await self.send_message(outmsg)

                # print('Message received', msg)
                current_time = time.time()

                # print("Time between messages", current_time - last_msg_time)

                # If we haven't heard a message in a given amount of time
                # terminate connection and event loop.
                if current_time - last_msg_time > self._timeout:
                    self.notify_message_listeners(MsgID.CONNECTION_CLOSED, 0)
                    self.stop()

                # update the time of the last message
                last_msg_time = current_time

                dispatch_message(self, msg)

        await self._shutdown_event_loop()

    def _start_event_loop(self):
        self._running = True
        loop = asyncio.get_event_loop()
        asyncio.ensure_future(self._dispatch_loop())
        if not loop.is_running():
            loop.run_forever()
        else:
            print("Loop is already running. This might be due to running '%gui async in a interactive shell")

    def start(self):
        """
        Starts an asynchronous event loop to receive and send messages. The
        loop runs until `self.stop` is called or the connection timeouts.

        """
        self._start_event_loop()

    async def _shutdown_event_loop(self):
        print('Shutting down event loop')
        loop = asyncio.get_event_loop()
        await loop.shutdown_asyncgens()
        loop.stop()

    def stop(self):
        """
        Closing WebSocket connection and shutdowns the event loop.

        All events received PRIOR to calling this function will be executed.
        """
        self._running = False

        print("Closing connection")

    async def send_message(self, msg):
        """
        Send to a MAVLink message to the drone.

        Args:
            msg: MAVLinkMessage to be sent to the drone
        """
        if self._ws is not None and self._ws.open:
            msg.pack(self._mav)
            buf = bytes(msg.get_msgbuf())
            # buf = msg.get_msgbuf()
            await self._ws.send(buf)

    async def send_long_command(self, command_type, param1, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
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
        msg = self._mav.command_long_encode(self._target_system, self._target_component, command_type, confirmation,
                                            param1, param2, param3, param4, param5, param6, param7)
        await self.send_message(msg)

    def arm(self):
        """
        Send an arm command.
        """
        asyncio.ensure_future(self.send_long_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1))

    def disarm(self):
        """
        Send a disarm command.
        """
        asyncio.ensure_future(self.send_long_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0))

    def take_control(self):
        mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  # tells system to use PX4 custom commands
        custom_mode = MainMode.PX4_MODE_OFFBOARD.value
        custom_sub_mode = 0  # not used for manual/offboard
        asyncio.ensure_future(
            self.send_long_command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mode, custom_mode, custom_sub_mode))

    def release_control(self):
        mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  # tells system to use PX4 custom commands
        custom_mode = MainMode.PX4_MODE_MANUAL.value
        custom_sub_mode = 0  # not used for manual/offboard
        asyncio.ensure_future(
            self.send_long_command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mode, custom_mode, custom_sub_mode))

    def cmd_attitude(self, yaw, pitch, roll, thrust):
        time_boot_ms = 0  # this does not need to be set to a specific time
        # TODO: convert the attitude to a quaternion
        q = [0, 0, 0, 0]
        mask = 0b00000111
        msg = self._mav.set_attitude_target_encode(time_boot_ms, self._target_system, self._target_component, mask, q,
                                                   0, 0, 0, thrust)
        asyncio.ensure_future(self.send_message(msg))

    def cmd_attitude_rate(self, yaw_rate, pitch_rate, roll_rate, thrust):
        time_boot_ms = 0  # this does not need to be set to a specific time
        q = [0, 0, 0, 0]
        mask = 0b10000000
        msg = self._mav.set_attitude_target_encode(time_boot_ms, self._target_system, self._target_component, mask, q,
                                                   roll_rate, pitch_rate, yaw_rate, thrust)
        asyncio.ensure_future(self.send_message(msg))

    def cmd_moment(self, roll_moment, pitch_moment, yaw_moment, thrust):
        time_boot_ms = 0  # this does not need to be set to a specific time
        q = [0.0, 0.0, 0.0, 0.0]
        mask = 0b10000000
        msg = self._mav.set_attitude_target_encode(time_boot_ms, self._target_system, self._target_component, mask, q,
                                                   roll_moment, pitch_moment, yaw_moment, thrust)
        asyncio.ensure_future(self.send_message(msg))

    def cmd_velocity(self, vn, ve, vd, heading):
        time_boot_ms = 0  # this does not need to be set to a specific time
        mask = (PositionMask.MASK_IGNORE_YAW_RATE.value | PositionMask.MASK_IGNORE_ACCELERATION.value |
                PositionMask.MASK_IGNORE_POSITION.value)
        msg = self._mav.set_position_target_local_ned_encode(time_boot_ms, self._target_system, self._target_component,
                                                             mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, 0, 0, 0, vn, ve,
                                                             vd, 0, 0, 0, heading, 0)
        asyncio.ensure_future(self.send_message(msg))

    def cmd_position(self, n, e, d, heading):
        time_boot_ms = 0  # this does not need to be set to a specific time
        # mask = PositionMask.MASK_IS_LOITER.value
        mask = (PositionMask.MASK_IGNORE_YAW_RATE.value | PositionMask.MASK_IGNORE_ACCELERATION.value |
                PositionMask.MASK_IGNORE_VELOCITY.value)
        msg = self._mav.set_position_target_local_ned_encode(time_boot_ms, self._target_system, self._target_component,
                                                             mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, n, e, d, 0, 0,
                                                             0, 0, 0, 0, heading, 0)
        asyncio.ensure_future(self.send_message(msg))

    def takeoff(self, n, e, d):
        # for mavlink to PX4 need to specify the NED location for landing
        # since connection doesn't keep track of this info, have drone send it
        # abstract away that part in the drone class
        time_boot_ms = 0  # this does not need to be set to a specific time
        mask = PositionMask.MASK_IS_TAKEOFF.value
        mask |= (PositionMask.MASK_IGNORE_YAW_RATE.value | PositionMask.MASK_IGNORE_YAW.value |
                 PositionMask.MASK_IGNORE_ACCELERATION.value | PositionMask.MASK_IGNORE_VELOCITY.value)
        msg = self._mav.set_position_target_local_ned_encode(time_boot_ms, self._target_system, self._target_component,
                                                             mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, n, e, d, 0, 0,
                                                             0, 0, 0, 0, 0, 0)
        asyncio.ensure_future(self.send_message(msg))

    def land(self, n, e):
        # for mavlink to PX4 need to specify the NED location for landing
        # since connection doesn't keep track of this info, have drone send it
        # abstract away that part in the drone class
        d = 0  # going to land, so just set d to 0
        time_boot_ms = 0  # this does not need to be set to a specific time
        mask = PositionMask.MASK_IS_LAND.value
        mask |= (PositionMask.MASK_IGNORE_YAW_RATE.value | PositionMask.MASK_IGNORE_YAW.value |
                 PositionMask.MASK_IGNORE_ACCELERATION.value | PositionMask.MASK_IGNORE_VELOCITY.value)
        msg = self._mav.set_position_target_local_ned_encode(time_boot_ms, self._target_system, self._target_component,
                                                             mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, n, e, d, 0, 0,
                                                             0, 0, 0, 0, 0, 0)
        asyncio.ensure_future(self.send_message(msg))

    def set_home_position(self, lat, lon, alt):
        asyncio.ensure_future(self.send_long_command(mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, lat, lon, alt))

    def local_position_target(self, n, e, d, t=0):
        time_boot_ms = t
        mask = 0b1111111111111000
        msg = self._mav.position_target_local_ned(time_boot_ms, mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, n, e, d, 0,
                                                  0, 0, 0, 0, 0, 0, 0)
        asyncio.ensure_future(self.send_message(msg))

    def local_velocity_target(self, vn, ve, vd, t=0):
        time_boot_ms = t
        mask = 0b1111111111000111
        msg = self._mav.position_target_local_ned(time_boot_ms, mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, 0, 0, 0, vn,
                                                  ve, vd, 0, 0, 0, 0, 0)
        asyncio.ensure_future(self.send_message(msg))

    def local_acceleration_target(self, an, ae, ad, t=0):
        time_boot_ms = t
        mask = 0b1111111000111111
        msg = self._mav.position_target_local_ned(time_boot_ms, mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, 0, 0, 0, 0,
                                                  0, 0, an, ae, ad, 0, 0)
        asyncio.ensure_future(self.send_message(msg))

    def attitude_target(self, roll, pitch, yaw, t=0):
        time_boot_ms = t
        mask = 0b11111110
        frame_msg = mt.FrameMessage(0.0, roll, pitch, 0.0)
        q = [frame_msg.q0, frame_msg.q1, frame_msg.q2, frame_msg.q3]
        msg = self._mav.attitude_target(time_boot_ms, mask, q, 0, 0, 0, 0)
        asyncio.ensure_future(self.send_message(msg))

    def body_rate_target(self, p, q, r, t=0):
        time_boot_ms = t
        mask = 0b00011111
        quat = [0, 0, 0, 0]
        msg = self._mav.attitude_target(time_boot_ms, mask, quat, p, q, r, 0)
        asyncio.ensure_future(self.send_message(msg))
