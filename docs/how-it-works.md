---
id: how-it-works
title: How It Works
sidebar_label: Overview
---

The API provides a means of interaction with a target drone, this may be physical drone (not all hardware is supported), or a simulated drone as is the case for the Flying Car Nanodegree.

The core of the API is the [`Connection`](https://github.com/udacity/udacidrone/blob/master/udacidrone/connection/connection.py#L13) interface. All connections must implement this interface. A `Connection` is reponsible for sending/receiving messages, as well as serialization. Note a `Connection` does not store any data, it simply acts as a intermediary relaying sensor data and commands to their respective target. The messages can handled in a synchronous or asynchronous manner. See [`MavlinkConnection`](https://github.com/udacity/udacidrone/blob/master/udacidrone/connection/mavlink_connection.py#L17) for a sync I/O example and [`WebSocketConnection`](https://github.com/udacity/udacidrone/blob/master/udacidrone/connection/websocket_connection.py#L28) for an async I/O example.

The [`Drone`](https://github.com/udacity/udacidrone/blob/master/udacidrone/drone.py#L9) class's core function is being a storage container for sensor data, i.e. the current state of the drone. Similar to the `Connection` interface, the `Drone` is not meant to be used directly but rather as a base for additional functionality, such as callbacks will built upon:

```python
class MyDrone(Drone):
    # your code ...
```

