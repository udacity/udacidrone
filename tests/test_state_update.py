"""
Test updates to the `Drone` class given a message
"""
from udacidrone import Drone
from udacidrone.connection.mavlink_utils import dispatch_message
from udacidrone.connection import Connection

def test0():
    dummy_conn = Connection()