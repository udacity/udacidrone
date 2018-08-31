---
id: platform-px4
title: UdaciDrone with PX4
sidebar_label: PX4
---

Here is an overview of the functionality that UdaciDrone provides with a [PX4](http://px4.io/) drone using the [Mavlink](https://mavlink.io/en/) protocol.  For more specific details on the implementation of the communication with PX4 over Mavlink, check out the `mavlink_connection.py` and `mavlink_utils.py` classes.

This document discuses the following:
 - [general PX4 configuration](#general-px4-setup)
 - [mavlink connection with PX4](#mavlinkconnection-overview)

## General PX4 Setup ##

These are a set of general setup instructions.  Note that your platform may have slightly different instructions based on how the PX4 flight stack is setup and installed.

*TODO: add instructions using the Intel Aero as an example*

## `MavlinkConnection` Overview ##

TODO: instructions for using the PX4Connection class, for both a shell version and a script version.

The connection takes slightly different inputs depending on whether the connection is to be used in the Python Shell or in a Python Script.

### Address ###

One of the main inputs to the `MavlinkConnection` is the "address" of the drone.  If you are sending commands over the wireless link, this would be the serial port and baud rate to use for the communication, while if you are on something like the Intel Aero, this would be UDP address to the UDP stream of the connection to the PX4 autopilot.

Therefore this is really platform dependent and we can only list some of the popular options, you may need to look into the documentation of your specific drone for this.

### Python Shell ###

For use in a python shell:

```py
>>> conn = PX4Connection('<serial port and baud rate>', threaded=True)
```

### Python Script ###

For use in a python script:

```py
>>> conn = PX4Connection('<serial port and baud rate>', threaded=False)
```
