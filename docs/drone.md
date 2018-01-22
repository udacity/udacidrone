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
`cmd_position(north, east, down, heading)` | command the drone to move to a specific NED defined position (in meters) with a specific heading (in radians)
`cmd_velocity(velocity_north, velocity_east, velocity_down, heading)` | command the drone to have the specified current velocity (in meters/second) and heading (in radians)
`cmd_attitude(roll, pitch, yawrate, thrust)` | command the drone to the specified attitude, where roll and pitch are in radian, yawrate is the desired rate of heading change in radian/second and thrust is the desired vertical acceleration in meters/second^2
`cmd_attitude_rate(roll_rate, pitch_rate, yaw_rate, thrust)` | command the drone to have the specified attitude rates in radians/second and thrust (vertical acceleration) in meters/second^2
`cmd_moment(roll_moment, pitch_moment, yaw_moment, thrust)` | command a moment in Newtom*meters and a vertical force in Netwons
`cmd_motors(motor_rpm)` | command the exact RPM of the motors from a vector of 4 inputs bounded between 0 and 1 (TODO: check this)
`set_home_position(longitude, latitude, altitude)` | set the GPS home position for the drone.  This changes the origin point of the local NED frame and therefore adjusts the local position information.
`start_log(directory, name)` | start logging telemetry data to the specified directory with the specified filename.
`stop_log()` | stop logging telemetry data
