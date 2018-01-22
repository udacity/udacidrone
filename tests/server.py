"""
Dummy mavlink server to test sending/receiving messages
"""
import asyncio
import signal
# import pytest
from io import BytesIO

import numpy as np
import websockets

from pymavlink.dialects.v20 import ardupilotmega as mavlink
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401


# if platform.system() is not 'Windows':
#     import uvloop
#     asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())
# logger = logging.getLogger('websockets.server')
# logger.setLevel(logging.ERROR)
# logger.addHandler(logging.StreamHandler())
async def ws_server(host, port, stop):
    print("Starting WebSocket server @ {0}:{1}".format(host, port))
    await websockets.serve(ws_receive_msgs, host, port)
    # async with websockets.serve(ws_receive_msgs, host, port):
    #     await stop


async def ws_receive_msgs(ws, path):
    f = BytesIO()
    mav = mavlink.MAVLink(f)
    while True:
        try:
            # hb = mav.heartbeat_encode(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
            #                           mavutil.mavlink.MAV_STATE_ACTIVE)
            # ws.send(hb)
            msg = await ws.recv()
        except Exception as e:
            print(e)
        else:
            try:
                dm = mav.decode(bytearray(msg))
            except mavlink.MAVError as e:
                print(dm)
                print(e)


async def ws_client(host, port, stop):
    uri = 'ws://{0}:{1}'.format(host, port)
    print('Starting WebSocket client @', uri)
    c = WebSocketConnection(uri)
    d = Drone(c)
    d.start()
    # buffer to make sure connection is active
    await asyncio.sleep(0.1)
    print("Connection open?", c.open)
    while not stop.done():
        # bombard server with drone operations
        north = np.random.uniform(-1e7, 1e7)
        east = np.random.uniform(-1e7, 1e7)
        down = np.random.uniform(-1e5, 0)
        yaw = np.radians(np.random.uniform(-180, 180))
        roll = np.radians(np.random.uniform(-180, 180))
        pitch = np.radians(np.random.uniform(-180, 180))
        thrust = np.random.uniform(-1e4, 1e4)
        yaw_rate = np.radians(np.random.uniform(-180, 180))
        roll_rate = np.radians(np.random.uniform(-180, 180))
        pitch_rate = np.radians(np.random.uniform(-180, 180))
        north_velocity = np.random.uniform(-1e2, 1e2)
        east_velocity = np.random.uniform(-1e2, 1e2)
        down_velocity = np.random.uniform(-1e2, 0)
        # rpm = np.random.uniform(0, 1e3)

        d.arm()
        d.take_control()
        d.set_home_position(north, east, -down)
        d.cmd_position(north, east, down, yaw)
        d.takeoff(down)
        d.cmd_attitude(roll, pitch, yaw_rate, thrust)
        d.cmd_attitude_rate(roll_rate, pitch_rate, yaw_rate, thrust)
        d.cmd_velocity(north_velocity, east_velocity, down_velocity, yaw)
        # d.cmd_motors(rpm)  # not implemented
        d.release_control()
        d.disarm()
        await asyncio.sleep(0.1)


# coro = asyncio.start_server(handle_echo, '127.0.0.1', 8888, loop=loop)
async def handle_echo(reader, writer):
    f = BytesIO()
    mav = mavlink.MAVLink(f)

    data = await reader.read(1024)
    msg = data.decode()
    try:
        mav.decode(bytearray(msg))
    except mavlink.MAVError as e:
        print(e)


async def tcp_server(host, port):
    print("Starting TCP server @ {0}:{1}".format(host, port))
    loop = asyncio.get_event_loop()
    await asyncio.start_server(handle_echo, '127.0.0.1', 8888, loop=loop)


async def f():
    host = '127.0.0.1'
    port = 3002
    stop = asyncio.Future()
    asyncio.get_event_loop().add_signal_handler(signal.SIGALRM, stop.set_result, None)

    signal.alarm(2)
    await ws_server(host, port, stop)
    await ws_client(host, port, stop)


def test_websocket_sending():
    loop = asyncio.get_event_loop()
    loop.run_until_complete(f())
