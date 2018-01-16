"""
Relay websocket messages between the FCND Unity Simulator
and a python client.

This must be done due to the current limitation of C# System.Net
not being usable in a WebGL build.

Instructions:
    - Run this file prior to running the simulator scene.
    - In the simulator scene make sure the transport protocol is set to WebSocket.
    - Run the simulator scene.
    - Run a file using a WebSocketConnection, such as test_websocket_connection.py
      in this directory.
"""
import platform
import asyncio
import logging
import signal
import time
from collections import Counter
from io import BytesIO

import uvloop
import websockets
from pymavlink.dialects.v20 import ardupilotmega as mavlink

if platform.system() is not 'Windows':
    import uvloop
    asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())
logger = logging.getLogger('websockets.server')
logger.setLevel(logging.ERROR)
logger.addHandler(logging.StreamHandler())

HOST = '127.0.0.1'
PORT = 5760

f = BytesIO()
mav = mavlink.MAVLink(f)

# Keep track of connections. We'll use this to relay
# a connection's messages to all the other connections.
connections = set()
message_rates = Counter()


async def relay(ws, path):
    global connections
    global message_rates
    connections.add(ws)
    prev_time = time.time()
    while True:
        try:
            msg = await ws.recv()
        except websockets.ConnectionClosed:
            print('Connection closed, remaining connected clients', len(connections))
            connections.remove(ws)
        else:
            dm = mav.decode(bytearray(msg))
            mt = dm.get_type()
            now = time.time()
            message_rates[mt] += 1
            diff = now - prev_time
            if diff > 1.0:
                for mt in message_rates:
                    v = message_rates[mt] / diff
                    print('Message rate for msg {0} is {1} Hz'.format(mt, v))
                print()
                prev_time = now
                message_rates.clear()
            for conn in connections:
                if conn != ws:
                    await conn.send(msg)


async def server(stop):
    print("Starting WebSocket server @ {0}:{1}".format(HOST, PORT))
    async with websockets.serve(relay, HOST, PORT):
        await stop


if __name__ == '__main__':
    loop = asyncio.get_event_loop()

    # The stop condition is set when receiving SIGTERM.
    stop = asyncio.Future()
    try:
        loop.add_signal_handler(signal.SIGTERM, stop.set_result, None)
    except NotImplementedError:
        print("this will result in an error on windows")
        if platform.system() is 'Windows':
            pass

    # Run the server until the stop condition is met.
    loop.run_until_complete(server(stop))
