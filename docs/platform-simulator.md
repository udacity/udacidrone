---
id: platform-simulator
title: UdaciDrone with the Unity Simulator
sidebar_label: Unity Simulator
---

Here is an overview of the functionality that UdaciDrone provides with a [PX4](http://px4.io/) drone using the [Mavlink](https://mavlink.io/en/) protocol.  For more specific details on the implementation of the communication with PX4 over Mavlink, check out the `mavlink_connection.py` and `mavlink_utils.py` classes.

This document discuses the following:
 - [general Unity simulator configuration](#general-unity-simulator-setup)
 - [mavlink connection with PX4](#mavlinkconnection-overview)

This document discuses the following:
 - general simulator configuration
 - overview of the `UnityConnection` class
 - platform specific limitations


## General Unity Simulator Setup ##

Nothing specific needs to be setup, will work out of the box!

## `MavlinkConnection` Overview ##

Here are examples of how to use the connection class in both the shell and in a script.

### Python Shell ###

```py
>>> conn = UnityConnection('tcp:127.0.0.1:5760', threaded=True)
```

### Python Script ###

```py
conn = UnityConnection('tcp:127.0.0.1:5760', threaded=False)
```