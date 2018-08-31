
Welcome to the UdaciDrone API!

[![CircleCI](https://circleci.com/gh/udacity/udacidrone.svg?style=svg)](https://circleci.com/gh/udacity/udacidrone)

## Overview ##

This is the Udacity Drone Python API. It provides a protocol agnostic API for communicating with a quadcopter, be it in the simulators provided in the Flying Car Nanodegree Program or even some real drones.

The API is designed in two parts: a `Drone` class and a set of connection classes.  The `Drone` class provides a representation of the physical or simulated drone and exposes the core API enabling interaction with the connected drone.  The connection classes are protocol specific implementations of the abstract `Connection` class, enabling the `Drone` to be configured to communicate over several different protocols used with drones today.

For those looking to just dive right in to using the UdaciDrone API, [check out our Getting Started guide](https://udacity.github.io/udacidrone/docs/getting-started.html).

## Drone ##

The core element of the UdaciDrone API is the Drone class.  This drone class is a representation of the physical or simulated drone you are connect to.  Through this Drone class, you are able to retrieve state information and send various commands.  Most importantly, `Drone` is communication protocol independent, which means any code you write interacting with `Drone` will work on any simulator or drone who's protocol has a `Connection` implementation!

For a detailed understanding of the `Drone` class, check out the detailed [Drone](https://udacity.github.io/udacidrone/docs/drone-api.html) documentation.

## Connection ##

These set of classes contain implementations of specific communication protocols over which `Drone` can connect with a real drone or simulator.

### Currently Supported Protocols ###

While the `Drone` API is designed to be protocol agnostic, in order to communicate with different types of drones, an implementation of each desired protocol needs to be built in the backend.  Currently, UdaciDrone supports the following communication protocols:

 - the [Mavlink Protocol](https://mavlink.io/en/) used in the [Dronecode](https://www.dronecode.org/) community.

 - [cflib](https://github.com/bitcraze/crazyflie-lib-python) the API used for [Bitcraze's crazyflie](https://www.bitcraze.io/crazyflie-2/)

This means that this API currently works with the [Udacity Simulator](https://github.com/udacity/FCND-Simulator-Releases/releases/tag/0.0.1) any [PX4](http://px4.io/) powered drone, and the crazyflie!.

For a detailed understanding of the `Connection` class, check out the detailed [Connection](https://udacity.github.io/udacidrone/docs/connection-api.html) documentation.

## More Details ##

For more detailed documentation, [check out our docs page](https://udacity.github.io/udacidrone/).
