
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
import uvloop
import asyncio
import signal
import websockets
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from io import BytesIO

asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())

HOST = '127.0.0.1'
PORT = 5760

f = BytesIO()
mav = mavlink.MAVLink(f)

# Keep track of connections. We'll use this to relay
# a connection's messages to all the other connections.
connections = set()


async def relay(ws, path):
    global connections
    connections.add(ws)
    while True:
        try:
            msg = await ws.recv()
        except websockets.ConnectionClosed:
            print('Connection closed, remaining connected clients', len(connections))
            connections.remove(ws)
        else:
            print('Message to relay', msg)
            for conn in connections:
                if conn != ws:
                    await conn.send(msg)


async def server(stop):
    async with websockets.serve(relay, HOST, PORT):
        await stop


if __name__ == '__main__':
    loop = asyncio.get_event_loop()

    # The stop condition is set when receiving SIGTERM.
    stop = asyncio.Future()
    loop.add_signal_handler(signal.SIGTERM, stop.set_result, None)

    # Run the server until the stop condition is met.
    loop.run_until_complete(server(stop))
