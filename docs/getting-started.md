---
id: getting-started
title: Getting Started
sidebar_label: Getting Started
---

This a short guide with snippets on how to install UdaciDrone and get familiar with using some of the key parts to the UdaciDrone API.

To be able to demonstrate how the UdaciDrone API works, we'll need ourselves a fake drone.  To get this part, [download Udacity's drone simulator](https://github.com/udacity/FCND-Simulator-Releases/releases/tag/0.0.1) so you can run some of the example code below!

## Requirements ##

This API requires Python 3, and depends on the following packages:

 - numpy
 - matplotlib
 - jupyter
 - lxml
 - pymavlink
 - pyserial
 - websockets
 - utm

To simplify installing all the dependencies and this library, you can install the [Python Starter Kit](https://github.com/udacity/FCND-Term1-Starter-Kit) provided for the Udacity Flying Car Nanodegree.

Or you can manually install all the above dependencies and then simply install udacidrone using `pip install`:

```sh
pip install udacidrone
```

## Using UdaciDrone ##

The UdaciDrone API can be used either in Python scripts or it can be used in the Python terminal itself.  For this Getting Started Guide, we will be walking through an example using the UdaciDrone API in a Python terminal paired with the [Udacity Simulator](https://github.com/udacity/FCND-Simulator-Releases/releases/tag/0.0.1).

There are two main parts to the UdaciDrone API, the `Drone` and different types of connections.  For now, the main connection type that will be used is the `MavlinkConnection`.

```python
>>> from udacidrone import Drone
>>> from udacidrone.connection import MavlinkConnection
```

These are the imports that will let you get started working with a drone that is connected using the mavlink communication protocol.

## Setting Up a Connection ##

Before we can have a `Drone`, we first must create a connection object which will be used by `Drone` to communicate with the actual drone.

```python
>>> conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=True)
```

Note: If you are not running this in a python terminal (e.g. a simply script), the `threaded=True` parameter is not needed.  For more detail on what this parameter does [check out the connection api documentation](connection.md)

## Setting Up the Drone ##

Once we have a connection set up, we're ready to set up our `Drone`!  We just need to tell it over which connection it is receiving information and it is ready to go.

```python
>>> drone = Drone(conn)
```

In order to start controlling it, etc, we do need to start the connection between our code and the drone itself:

```python
>>> drone.start()
```


## Example ##

So now we have something that looks a little like this:

```python
>>> from udacidrone import Drone
>>> from udacidrone.connection import MavlinkConnection
>>> conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=True)
>>> drone = Drone(conn)
>>> drone.start()
```

So now what can we do?

Well we would love to be able to takeoff, however before we can take off, we first much get the drone ready for takeoff.  This is a process that requires both taking control of the drone and arming it for takeoff.

```python
>>> drone.take_control()
>>> drone.arm()
```

Now we are ready to send some more fun commands, for example if we run:

```python
>>> drone.takeoff(3)
```

you should see the drone takeoff to 3 meters above the ground in the simulator!

To see all the other commands you have available to you and to further explore the Drone API [check out the Drone API's detailed documentation](drone.md)!

## Message Logging

The telemetry data is automatically logged in "Logs\TLog.txt. Each row contains a comma separated representation of each message. The first row is the incoming message type. The second row is the time. The rest of the rows contain all the message properties. 

### Reading Telemetry Logs

Logs can be read using:

```python
t_log = drone.read_telemetry_data(filename)
```

The data is stored as a dictionary of message types. For each message type, there is a list of numpy arrays. For example, to access the longitude and latitude from a `global_position_msg`:

```python
# Time is always the first entry in the list
time = t_log['global_position_msg'][0][:]
longitude = t_log['global_position_msg'][1][:]
latitude = t_log['global_position_msg'][2][:]
```

The data between different messages will not be time synced since they are recorded at different times.