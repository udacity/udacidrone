---
id: welcome
title: Welcome
sidebar_label: Welcome
---

Welcome to the UdaciDrone API!


## Overview ##

The UdaciDrone API is designed as a communication protocol independent python API for command and control of a drone.


TODO: add cool image...

The API is designed in two parts: a `Drone` class and a `Connection` class.
The `Drone` class provides a representation of the physical or simulated drone enabling interaction with the connected drone.  While the `Connection` class is an abstract class implemented for each desired communication protocol to support.

Note: this is designed to be used with the idea of event driven programming.

TODO: link to some event driven programming resources!

For those looking to just dive right in to using the UdaciDrone API, [check out our Getting Started guide](getting-started.md).

## Drone ##

The core element of the UdaciDrone API is the Drone class.  This drone class is a representation of the physical or simulated drone you are connect to.  Through this Drone class, you are able to retrieve state information and send various commands.

For a detailed understanding of the `Drone` class, check out the detailed [Drone](drone.md) documentation.

## Connection ##

For a detailed understanding of the `Connection` class, check out the detailed [Connection](connection.md) documentation.