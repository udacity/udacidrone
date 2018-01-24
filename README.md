
Welcome to the UdaciDrone API!

[![CircleCI](https://circleci.com/gh/udacity/udacidrone.svg?style=svg)](https://circleci.com/gh/udacity/udacidrone)

## Overview ##

This is the Udacity Drone Python API. It provides an interface for communicating with your quadcopter in the simulators provided in the Flying Car Nanodegree Program, and for communicating with a real drone if you wish to do so.


TODO: add structure overview image...

The API is designed in two parts: a `Drone` class and a set of connection classes.  The `Drone` class provides a representation of the physical or simulated drone enabling interaction with the connected drone.  The connection classes are protocol specific implementations of the abstract `Connection` class, enabling the `Drone` to be configured to communicate over several different protocols used with drones today.

For those looking to just dive right in to using the UdaciDrone API, [check out our Getting Started guide](https://udacity.github.io/udacidrone/docs/getting-started.html).

## Drone ##

The core element of the UdaciDrone API is the Drone class.  This drone class is a representation of the physical or simulated drone you are connect to.  Through this Drone class, you are able to retrieve state information and send various commands.  Most importantly, `Drone` is communication protocol independent, which means any code you write interacting with `Drone` will work on any simulator or drone who's protocol has a `Connection` implementation!

For a detailed understanding of the `Drone` class, check out the detailed [Drone](https://udacity.github.io/udacidrone/docs/drone-api.html) documentation.

## Connection ##

These set of classes contain implementations of specific communication protocols over which `Drone` can connect with a real drone or simulator.  Currently, the only supported protocol is the [Mavlink Protocol](https://mavlink.io/en/) used in the [Dronecode](https://www.dronecode.org/) community.  This means that this API currently only works with the [Udacity Simulator](https://github.com/udacidrone/FCND-Simulator-Releases/releases/tag/0.0.1) and any [PX4](http://px4.io/) powered drone!.

For a detailed understanding of the `Connection` class, check out the detailed [Connection](https://udacity.github.io/udacidrone/docs/connection-api.html) documentation.

## Reference Frames ##

Two different reference frames are defined and used within the Drone API. Global positions are defined as [longitude, latitude, altitude (positive up)]. Local reference frames are defined [North, East, Down (positive up)] and is relative to a nearby global home provided. Both reference frames are defined in a proper right-handed reference frame. The global reference frame is what is provided by the Drone's GPS. Two convenience functions, `global_to_local()` and `local_to_global()` are provided within the `frame_utils.py` script to convert between the two frames. These functions are wrappers on `utm` library functions.


