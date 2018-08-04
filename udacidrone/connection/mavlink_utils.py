from enum import Enum

from pymavlink import mavutil

from udacidrone.messaging import MsgID

from . import message_types as mt


class ConnectionType(Enum):
    """
    Different possible connection types.
    NOTE: Right now the only implemented type is PX4 mavlink
    """
    MAVLINK_PX4 = 1
    # MAVLINK_APM = 2
    # PARROT = 3
    # DJI = 4


class MainMode(Enum):
    """Constant which isn't defined in Mavlink but is useful for PX4"""
    PX4_MODE_MANUAL = 1
    PX4_MODE_OFFBOARD = 6


class PlaneMode(Enum):
    """Constant which isn't defined in Mavlink but useful when dealing with
    the airplane simulation"""
    SUB_MODE_MANUAL = 1
    SUB_MODE_LONGITUDE = 2
    SUB_MODE_LATERAL = 3
    SUB_MODE_STABILIZED = 4


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


class AttitudeMask(Enum):
    MASK_IGNORE_ATTITUDE = 0b10000000
    MASK_IGNORE_RATES = 0b00000111


def dispatch_message(conn, msg):

    # parse out the message based on the type and call
    # the appropriate callbacks

    # http://mavlink.org/messages/common/#GLOBAL_POSITION_INT
    if msg.get_type() == 'GLOBAL_POSITION_INT':
        timestamp = msg.time_boot_ms / 1000.0
        # parse out the gps position and trigger that callback
        gps = mt.GlobalFrameMessage(timestamp, float(msg.lat) / 1e7, float(msg.lon) / 1e7, float(msg.alt) / 1000)
        conn.notify_message_listeners(MsgID.GLOBAL_POSITION, gps)

        # parse out the velocity and trigger that callback
        vel = mt.LocalFrameMessage(timestamp, float(msg.vx) / 100, float(msg.vy) / 100, float(msg.vz) / 100)
        conn.notify_message_listeners(MsgID.LOCAL_VELOCITY, vel)

    # http://mavlink.org/messages/common/#HEARTBEAT
    elif msg.get_type() == 'HEARTBEAT':
        timestamp = 0.0
        motors_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

        # TODO: determine if want to broadcast all current mode types
        # not just boolean on manual
        guided_mode = False

        # extract whether or not we are in offboard mode for PX4
        # (the main mode)
        main_mode = (msg.custom_mode & 0x000F0000) >> 16
        if main_mode == MainMode.PX4_MODE_OFFBOARD.value:
            guided_mode = True

        state = mt.StateMessage(timestamp, motors_armed, guided_mode, msg.system_status)
        conn.notify_message_listeners(MsgID.STATE, state)

    # http://mavlink.org/messages/common#LOCAL_POSITION_NED
    elif msg.get_type() == 'LOCAL_POSITION_NED':
        timestamp = msg.time_boot_ms / 1000.0
        # parse out the local positin and trigger that callback
        pos = mt.LocalFrameMessage(timestamp, msg.x, msg.y, msg.z)
        conn.notify_message_listeners(MsgID.LOCAL_POSITION, pos)

        # parse out the velocity and trigger that callback
        vel = mt.LocalFrameMessage(timestamp, msg.vx, msg.vy, msg.vz)
        conn.notify_message_listeners(MsgID.LOCAL_VELOCITY, vel)

    # http://mavlink.org/messages/common#HOME_POSITION
    elif msg.get_type() == 'HOME_POSITION':
        timestamp = 0.0
        home = mt.GlobalFrameMessage(timestamp,
                                     float(msg.latitude) / 1e7,
                                     float(msg.longitude) / 1e7,
                                     float(msg.altitude) / 1000)
        conn.notify_message_listeners(MsgID.GLOBAL_HOME, home)

    # http://mavlink.org/messages/common/#SCALED_IMU
    elif msg.get_type() == 'SCALED_IMU':
        timestamp = msg.time_boot_ms / 1000.0
        # break out the message into its respective messages for here
        accel = mt.BodyFrameMessage(timestamp, msg.xacc / 1000.0, msg.yacc / 1000.0, msg.zacc / 1000.0)  # units -> [mg]
        conn.notify_message_listeners(MsgID.RAW_ACCELEROMETER, accel)

        gyro = mt.BodyFrameMessage(timestamp,
                                   msg.xgyro / 1000.0,
                                   msg.ygyro / 1000.0,
                                   msg.zgyro / 1000.0)  # units -> [millirad/sec]
        conn.notify_message_listeners(MsgID.RAW_GYROSCOPE, gyro)

    # http://mavlink.org/messages/common#SCALED_PRESSURE
    elif msg.get_type() == 'SCALED_PRESSURE':
        timestamp = msg.time_boot_ms / 1000.0
        pressure = mt.BodyFrameMessage(timestamp, 0, 0, msg.press_abs)  # unit is [hectopascal]
        conn.notify_message_listeners(MsgID.BAROMETER, pressure)

    # http://mavlink.org/messages/common#DISTANCE_SENSOR
    elif msg.get_type() == 'DISTANCE_SENSOR':
        timestamp = msg.time_boot_ms / 1000.0
        direction = 0
        # TODO: parse orientation
        # orientation = msg.orientation
        meas = mt.DistanceSensorMessage(timestamp,
                                        float(msg.min_distance) / 100,
                                        float(msg.max_distance) / 100, direction,
                                        float(msg.current_distance) / 100,
                                        float(msg.covariance) / 100)
        conn.notify_message_listeners(MsgID.DISTANCE_SENSOR, meas)

    # http://mavlink.org/messages/common#ATTITUDE_QUATERNION
    elif msg.get_type() == 'ATTITUDE_QUATERNION':
        timestamp = msg.time_boot_ms / 1000.0
        # TODO: check if mask notifies us to ignore a field

        fm = mt.FrameMessage(timestamp, msg.q1, msg.q2, msg.q3, msg.q4)
        conn.notify_message_listeners(MsgID.ATTITUDE, fm)

        gyro = mt.BodyFrameMessage(timestamp, msg.rollspeed, msg.pitchspeed, msg.yawspeed)
        conn.notify_message_listeners(MsgID.RAW_GYROSCOPE, gyro)

    # DEBUG
    elif msg.get_type() == 'STATUSTEXT':
        # print("[autopilot message] " + msg.text.decode("utf-8"))
        pass
