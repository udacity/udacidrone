---
id: platform-px4
title: UdaciDrone with PX4
sidebar_label: PX4
---

This will contain the overview information on how to use a PX4 drone with the UdaciDrone API.

Specifically should discuss how to use the `PX4Connection` class, a short version of what the parameters mean (the long version is in the files themselves if people are very curious), the supported list of commands, anything else specific to note that is related to the a PX4 drone and its behavior with UdaciDrone.

# UdaciDrone with a PX4 Drone #

This document discuses the following:
 - general PX4 configuration
 - overview of the `PX4Connection` class
 - platform specific limitations


## General PX4 Setup ##

These are a set of general setup instructions.  Note that your platform may have slightly different instructions based on how the PX4 flight stack is setup and installed.

TODO: These instructions maybe will also have some references for how to do things with the Intel Aero???
TODO: talk to Andy to see how much overlap can be had between this documentation and the stuff in the classroom.


## `PX4Connection` Overview ##

TODO: instructions for using the PX4Connection class, for both a shell version and a script version.


The connection takes slightly different inputs depending on whether the connection is to be used in the Python Shell or in a Python Script.

### Address ###

One of the main inputs to the `PX4Connection` is the "address" of the drone.  If you are sending commands over the wireless link, this would be the serial port and baud rate to use for the communication, while if you are on something like the Intel Aero, this would be UDP address to the UDP stream of the connection to the PX4 autopilot.

Therefore this is really platform dependent and we can only list some of the popular options, you may need to look into the documentation of your specific drone for this.

TODO: should also list all the supported connection types, since that's really the limiting factor for udacidrone...

TODO: I think the supported connection types are serial, UDP and TCP (we are using pymavlink behind the scenes, so it's basically a subset of what they support... should make sure it is the full set though).

### Python Shell ###

For use in a python shell:

```py
>>> conn = PX4Connection('<serial port and baud rate I think>', threaded=True)
```

### Python Script ###

For use in a python script:

```py
>>> conn = PX4Connection('<serial port and baud rate I think>', threaded=False)
```


## API Notes ##


## General Notes ##