---
id: connection-api
title: Connection API Overview
sidebar_label: Overview
---

TODO: this should be a brief overview of how the connection classes are structures.

Basically we have a `Connection` class that is really just an abstract class (depending on your background, you could also just think of it as an interface).

For each communication protocol our API wants to support, there needs to be a protocol specific class that subclasses the `Connection` class and implements all of the required functions in that protocol.

So really, when we say protocol independent, it's the `Drone` facing side of things that is protocol independent.  All the `Drone` attributes and functions that you would interact with when using UdaciDrone are protocol independent.  We achieve this by having our own set of attributes, etc, so each protocol needs to correctly parse messages and send information to the drone accordingly.

For example, with the mavlink protocol, things like global position and velocity are in the same message, but we have it broken out as two different attributes (and effectively two different internal messages -> from the connection class to the drone).


Currently only the Mavlink protocol is supported, but for those interested in extending the UdaciDrone API to support other protocols (e.g. Parrot or DJI), please refer to [this documentation](extending-connection.md) to see how to get started!

TODO: need to explain all of the input parameters, specifically explain when to use threaded and not threaded!!

TODO: also need to explain how the `Connection` class interacts with the 


## Messages ##

In order to have `Drone` be independent on the communication protocol, UdaciDrone uses an internal set of message names and types to pass data between a connection implementation and the `Drone`.

### Outgoing Message Types

Here is the set of available message names and the data they should carry.

Message Name | Data Type | Description
--- | --- | ---
`MsgID.STATE` | `StateMessage` | changes in either the armed state of the drone or the control state (namely whether or not the drone is configured to take external commands from a script)
`MsgID.GLOBAL_POSITION` | `GlobalFrameMessage` | new GPS position of the drone
`MsgID.LOCAL_POSITION` | `LocalFrameMessage` | new local NED position of the drone
`MsgID.GLOBAL_HOME` | `GlobalFrameMessage` | new GPS home position of the drone
`MsgID.LOCAL_VELOCITY` | `LocalFrameMessage` | new velocity vector of the drone
`MsgID.CONNECTION_CLOSED` | n/a | connection to the drone has been terminated.  This means that attributes will no longer be updated and all drone facing commands will not be executed by the drone.
`MsgID.RAW_GYROSCOPE` | `BodyFrameMessage` | new raw gyroscope values
`MsgID.RAW_ACCELEROMETER` | `BodyFrameMessage` | new raw accelerometer information
`MsgID.BAROMETER` | n/a | new barometric pressured based altitude information
`MsgID.DISTANCE_SENSOR` | `DistanceMessage` | new distance measurement information (e.g. from onboard sonar or stereo vision sensors)
`MsgID.ATTITUDE` | `BodyFrameMessage` | new attitude information
