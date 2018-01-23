---
id: drone-attributes
title: Drone Attributes
sidebar_label: Attributes
---

The attributes contain the state information of the connected drone.

Note that all attributes that are arrays are formatted as numpy arrays.

Here is the list and definition of all the attributes in the drone class:


Attribute | Format | Description
--- | --- | ---
`connected` | boolean | the connected state of the API to the drone
`armed` | boolean | whether or not the drone is currently `armed`.  `Armed` means that the drone's motors are running and ready to take inputs.  Until the drone is `armed`, no motor inputs will create any effects (the motors are effectively turned off until the drone is `armed`)
`guided` | boolean | whether or not the drone is currently in a `guided` mode.  `Guided` mode means that the python script has control of the drone.  Until the drone is put into `guided` mode, no commands sent to it from a script will be executed.
`attitude` | [roll (degree), pitch (degree), yaw (degree)] | the current attitude of the drone
`global_position` | [longitude (degree), latitude (degree), altitude (meter)] | the current GPS position of the drone
`global_home` | [longitude (degree), latitude (degree), altitude (meter)] | the GPS position of the "home" location of the drone 
`local_position` | [north (meter), east (meter), down (meter)] | the current local position of the drone.  Local position being defined as the NED position of the drone with respect to some (0,0,0) (the home position)
`local_velocity` | [vnorth (meter/second), veast (meter/second), vdown (meter/second)] | The current velocity vector of the drone in meters/second, represented in the local NED frame
`acceleration_raw` | [x (), y (), z ()] | a vector of the raw accelerations measurement of the drone in the body frame
`gyro_raw` | [x (), y (), z ()] | a vector of the raw gyroscope measurement of the drone in the body frame
`barometer` | alt (meter) | the current altitude of the drone (in meters) as measured solely by the barometer onboard a vehicle (or simulated)



## Listening to Attribute Changes ##

As new information about the drone is passed to `Drone` through the connection, various attributes will be updated.  Callbacks are functions that can be registered to be called when a specific set of attributes are updated.  There are two steps needed to be able to create and register a callback:

1. Create the callback function:

Each callback function you may want needs to be defined as a member function of your custom class that subclasses `Drone` that takes in only the `self` parameter.  For example, here is the definition of a callback method we may use for when the local position attribute is updated:

```python
class MyDrone(Drone):
    ...

    def local_position_callback(self):
        """ this is triggered when self.local_position contains new data """
        pass
```

2. Register the callback:

In order to have your callback function called when the appropriate attributes are updated, each callback needs to be registered.  This registration takes place in your class's `__init__()` function as shown below:

```python
class MyDrone(Drone):

    def __init__(self, connection):
        ...

        # register a callback on local position attribute changes
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
```

Since callback functions are only called when certain drone attributes are changed, the first parameter to the callback registration indicates for which attribute changes you want the callback to occur.  Here is a list of the different sets of attribute changes you can listen to:

Registration flag | Conditions for triggering callback function
--- | ---
`MsgID.ANY` | changes in any attribute
`MsgID.STATE` | changes in either `armed` or `guided`
`MsgID.GLOBAL_POSITION` | changes in `global_position`
`MsgID.LOCAL_POSITION` | changes in `local_position`
`MsgID.GLOBAL_HOME` | changes in `global_home`
`MsgID.LOCAL_VELOCITY` | changes in `local_velocity`
`MsgID.CONNECTION_CLOSED` | connection to the drone has been terminated.  This means that attributes will no longer be updated and all drone facing commands will not be executed by the drone.
`MsgID.RAW_GYROSCOPE` | changes in `gyro_raw`
`MsgID.RAW_ACCELEROMETER` | changes in `acceleration_raw`
`MsgID.BAROMETER` | changes in `barometer`
`MsgID.DISTANCE_SENSOR` | reserved
`MsgID.ATTITUDE` | changes in `attitude`