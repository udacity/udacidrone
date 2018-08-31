---
id: platform-simulator
title: Unity Simulator
sidebar_label: Unity Simulator
---

This will contain the overview information on how to use the unity simulator with the UdaciDrone API.

Specifically should discuss how to use the `UnityConnection` class, a short version of what the parameters mean (the long version is in the files themselves if people are very curious), the supported list of commands, anything else specific to note that is related to the simulator and its behavior with UdaciDrone.

# UdaciDrone with the Unity Simulator #

This document discuses the following:
 - general simulator configuration
 - overview of the `UnityConnection` class
 - platform specific limitations


## General Unity Simulator Setup ##

Note: I don't think there is anything specific that needs to be set up with the simulator....


## `UnityConnection` Overview ##

TODO: instructions for using the UnityConnection class, for both a shell version and a script version.

### Python Shell ###

```py
>>> conn = UnityConnection('tcp:127.0.0.1:5760', threaded=True)
```

### Python Script ###

```py
conn = UnityConnection('tcp:127.0.0.1:5760', threaded=False)
```

## API Notes ##


## General Notes ##