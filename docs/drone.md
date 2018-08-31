---
id: drone-api
title: Drone API Overview
sidebar_label: Overview
---

TODO: this should be a brief overview of the elements that make up the drone API.

Specifically the Drone class takes in a connection (this is how we abstract the communication protocol) that is a subclass of the abstract `Connection` class (the connection class can be seen like a contract or an interface -> all of the different implementations have to provide implementations of the functions listed)

Through the connection, the Drone class keeps it's state attributed updated to the most recent values.

Finally, through the Drone class, you can control the Drone's behavior (here the Drone ends up being a pass-through).

## Parameters ##

the input parameter is the connection through which to talk to the drone.  Again the connection part is how we are abstracting out the communication protocol from this drone API class.


## Attributes ##

The attributes of the `Drone` are the properties of greatest interest.  To see a detailed list of the attributes [see the more detailed documentation](drone-attributes.md).

One thing that is important to note in this overview is that since this is all about the idea of event driven programming, you can register callbacks for changes in the `Drone` attributes.  [You can find more detail and examples here](drone-attributes.md).

## Commands ##


Finally, to command the drone, there are a set of command functions:

Function | Description
--- | ---
`start()` | start the connection to the drone (be it simulated or real)
`stop()` | stop the connection to the drone
`take_control()` | take control of the drone -> this configures the drone to start accepting different commands from Python
`release_control()` | give up control of the drone -> this configures the drone to stop accepting commands from Python and return to accepting manual user input
`arm()` | arm the drone to put it in a state ready for takeoff.  Until the drone is armed, no motor related commands will be executed
`disarm()` | disarm the drone to put it in a state with the motors effectively turned off
`takeoff(target_altitude)` | command the drone to takeoff to the specified altitude in meters
`land()` | command the drone to land at the current position
`cmd_position(north, east, altitude, heading)` | command the drone to move to a specific (N, E, altitude) defined position (in meters) with a specific heading (in radians)
`cmd_velocity(velocity_north, velocity_east, velocity_down, heading)` | command the drone to have the specified current velocity (in meters/second) and heading (in radians)
`cmd_attitude(roll, pitch, yawrate, thrust)` | command the drone to the specified attitude, where roll and pitch are in radian, yawrate is the desired rate of heading change in radian/second and thrust is the desired vertical acceleration in meters/second^2
`cmd_attitude_rate(roll_rate, pitch_rate, yaw_rate, thrust)` | command the drone to have the specified attitude rates in radians/second and thrust (vertical acceleration) in meters/second^2
`cmd_moment(roll_moment, pitch_moment, yaw_moment, thrust)` | command a moment in Newtom*meters and a vertical force in Netwons
`set_home_position(longitude, latitude, altitude)` | set the GPS home position for the drone.  This changes the origin point of the local NED frame and therefore adjusts the local position information.
`start_log(directory, name)` | start logging telemetry data to the specified directory with the specified filename.
`stop_log()` | stop logging telemetry data

## Logging ##

The `Drone` will also log telemetry data while a connection to a drone exists.  When starting the drone manually from a Python/iPython shell you have the option to provide a desired filename for the telemetry log file (such as "TLog-manual.txt" as shown belo).  This allows you to customize the telemetry log name as desired to help keep track of different types of log files you might have.

```python
>>> from drone import Drone
>>> drone = Drone()
>>> drone.start(threaded=True, tlog_name="TLog-manual.txt")
```

If `threaded` is set to `False`, the code will block and the drone logging can only be stopped by terminating the simulation. If the connection is threaded, the drone can be commanded using the commands described above, and the connection can be stopped (and the log properly closed) using:

```python
drone.stop()
```

### Message Logging

The telemetry data is automatically logged in "Logs\TLog.txt. Each row contains a comma separated representation of each message. The first row is the incoming message type. The second row is the time. The rest of the rows contain all the message properties. 

#### Reading Telemetry Logs

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


## Reference Frames ##

Two different reference frames are defined and used within the Drone API. Global positions are defined as [longitude, latitude, altitude (positive up)]. Local reference frames are defined [North, East, Down (positive down)] and is relative to a nearby global home provided. Both reference frames are defined in a proper right-handed reference frame. The global reference frame is what is provided by the Drone's GPS. Two convenience functions, `global_to_local()` and `local_to_global()` are provided within the `frame_utils.py` script to convert between the two frames. These functions are wrappers on `utm` library functions.