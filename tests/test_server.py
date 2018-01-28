"""
Dummy mavlink server to test sending/receiving messages
"""
import asyncio
import signal
import time
from enum import Enum
# import traceback
from io import BytesIO

import numpy as np
import pytest
import websockets

from pymavlink.dialects.v20 import ardupilotmega as mavlink
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401

TIMEOUT = 1
HOST = '127.0.0.1'
PORT = 3002

# asyncio.Future used for cancelling servers/connections
stop = None


class ConnectionProtocol(Enum):
    WebSocket = 'ws'
    TCP = 'tcp'


def exception_handler(loop, context):
    global stop
    if not stop.done():
        # TODO: figure out how to print the traceback
        # traceback.print_exc()
        stop.set_exception(context["exception"])


async def ws_server():
    print("Starting WebSocket server @ {0}:{1}".format(HOST, PORT))
    await websockets.serve(handle_ws, HOST, PORT)


async def tcp_server():
    print("Starting TCP server @ {0}:{1}".format(HOST, PORT))
    loop = asyncio.get_event_loop()
    await asyncio.start_server(handle_tcp, HOST, PORT, loop=loop)


async def handle_ws(ws, path):
    global stop
    f = BytesIO()
    mav = mavlink.MAVLink(f)
    while not stop.done():
        # hb = mav.heartbeat_encode(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
        #                           mavutil.mavlink.MAV_STATE_ACTIVE)
        # ws.send(hb)
        msg = await ws.recv()
        mav.decode(bytearray(msg))


async def handle_tcp(r, w):
    global stop
    f = BytesIO()
    mav = mavlink.MAVLink(f)
    while not stop.done():
        # NOTE: Since not guaranteed we'll get each mavlink message
        # independently we have to manually separate them.
        msg = await r.read(1024)
        # print(len(msg), msg)
        idx = 0
        while idx < len(msg):
            # decode the packet (msg + 10 header + 2 crc), ignoring signature
            plen = msg[idx + 1] + 10 + 2
            mav.decode(bytearray(msg[idx:idx + plen]))
            idx += plen


async def run_client(drone):
    drone.start()
    await asyncio.sleep(0.05)  # make sure connection starts

    global stop
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

        drone.arm()
        drone.take_control()
        drone.set_home_position(north, east, -down)
        drone.cmd_position(north, east, down, yaw)
        drone.takeoff(down)
        drone.cmd_attitude(roll, pitch, yaw_rate, thrust)
        drone.cmd_attitude_rate(roll_rate, pitch_rate, yaw_rate, thrust)
        drone.cmd_velocity(north_velocity, east_velocity, down_velocity, yaw)
        # drone.cmd_motors(rpm)  # not implemented
        drone.release_control()
        drone.disarm()
        await asyncio.sleep(0.02)


async def f(connection_type):
    global stop
    stop = asyncio.Future()
    asyncio.get_event_loop().add_signal_handler(signal.SIGALRM, stop.set_result, None)
    asyncio.get_event_loop().set_exception_handler(exception_handler)

    if connection_type == ConnectionProtocol.WebSocket:
        await ws_server()
    elif connection_type == ConnectionProtocol.TCP:
        await tcp_server()

    await asyncio.sleep(0.05)

    if connection_type == ConnectionProtocol.WebSocket:
        conn = WebSocketConnection('{0}://{1}:{2}'.format(ConnectionProtocol.WebSocket.value, HOST, PORT))
    elif connection_type == ConnectionProtocol.TCP:
        conn = MavlinkConnection('{0}:{1}:{2}'.format(ConnectionProtocol.TCP.value, HOST, PORT), threaded=True)

    drone = Drone(conn)
    signal.alarm(TIMEOUT)
    await run_client(drone)


@pytest.fixture(scope="module", params=[ConnectionProtocol.WebSocket, ConnectionProtocol.TCP])
def connection_type(request):
    return request.param


def test_client(connection_type):
    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        then = time.time()
        loop.run_until_complete(f(connection_type))
        now = time.time()
        assert now - then > TIMEOUT, stop.exception()
    finally:
        loop.close()
