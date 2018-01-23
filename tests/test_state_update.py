"""
Test updates to the `Drone` class given a message
"""
import os
import time
from io import BytesIO

import numpy as np

from pymavlink.dialects.v20 import ardupilotmega as mavlink
from udacidrone import Drone
from udacidrone.connection import Connection
from udacidrone.connection.mavlink_utils import dispatch_message

# force use of mavlink v2.0
os.environ['MAVLINK20'] = '1'


def test_drone_state_update():
    f = BytesIO()
    mav = mavlink.MAVLink(f)
    c = Connection()
    d = Drone(c)

    # http://mavlink.org/messages/common/#HEARTBEAT
    # msg = mav.heartbeat_encode(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
    #                            mavutil.mavlink.MAV_STATE_ACTIVE)

    # http://mavlink.org/messages/common/#GLOBAL_POSITION_INT
    lat = 37.7749 * 1e7  # degrees * 1e7
    lon = 122.4194 * 1e7  # degrees * 1e7
    alt = 33.3 * 1000  # millimeters
    vx = 12.3 * 100  # m/s * 100
    vy = 14.3 * 100  # m/s * 100
    vz = 18.3 * 100  # m/s * 100
    hdg = 88 * 100  # degrees * 100
    msg = mav.global_position_int_encode(time.time(),
                                         int(lat), int(lon), int(alt), int(alt), int(vx), int(vy), int(vz), int(hdg))
    dispatch_message(c, msg)
    # NOTE: the order switch of longitude and latitude
    assert np.array_equal(d.global_position, np.array([float(lon) / 1e7, float(lat) / 1e7, float(alt) / 1000]))
    assert np.array_equal(d.local_velocity, np.array([float(vx) / 100, float(vy) / 100, float(vz) / 100]))

    # http://mavlink.org/messages/common#LOCAL_POSITION_NED
    x = 10.
    y = 20.
    z = 30.
    vx = 30.
    vy = 22.
    vz = 8.
    msg = mav.local_position_ned_encode(time.time(), x, y, z, vx, vy, vz)
    dispatch_message(c, msg)
    assert np.array_equal(d.local_position, np.array([x, y, z]))
    assert np.array_equal(d.local_velocity, np.array([vx, vy, vz]))

    # http://mavlink.org/messages/common#HOME_POSITION
    home_lat = 37.7749 * 1e7  # degrees * 1e7
    home_lon = 122.4194 * 1e7  # degrees * 1e7
    home_alt = 0. * 1000
    msg = mav.home_position_encode(home_lat, home_lon, home_alt, 0, 0, 0, 0, 0, 0, 0)
    dispatch_message(c, msg)
    expect = np.array([float(home_lon) / 1e7, float(home_lat) / 1e7, float(home_alt) / 1000])
    assert np.array_equal(d.global_home, expect)

    # http://mavlink.org/messages/common#ATTITUDE_QUATERNION
    # Calculate euler angles with http://quaternions.online/
    q1 = 0.56098553
    q2 = 0.43045933
    q3 = -0.09229596
    q4 = 0.70105738
    expect_euler = np.deg2rad(np.array([30, -45, 90]))
    rollspeed = 33
    pitchspeed = 88
    yawspeed = 47
    msg = mav.attitude_quaternion_encode(time.time(), q1, q2, q3, q4, rollspeed, pitchspeed, yawspeed)
    dispatch_message(c, msg)
    assert np.isclose(d.gyro_raw, np.array([rollspeed, pitchspeed, yawspeed])).all()
    assert np.isclose(d.attitude, expect_euler).all()

    # NOTE: Not using these on the simulator backend yet

    # http://mavlink.org/messages/common/#SCALED_IMU
    # xacc = 0
    # yacc = 0
    # zacc = 0
    # xgyro = 0
    # ygyro = 0
    # zgyro = 0
    # msg = mav.scaled_imu_encode(time.time(), xacc, yacc, zacc, xgyro, ygyro, zgyro, 0, 0, 0)
    # dispatch_message(c, msg)

    # http://mavlink.org/messages/common#SCALED_PRESSURE
    # msg = mav.scaled_pressure_encode()
    # dispatch_message(c, msg)

    # http://mavlink.org/messages/common#DISTANCE_SENSOR
    # msg = mav.distance_sensor_encode()
    # dispatch_message(c, msg)
