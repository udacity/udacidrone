---
id: platform-crazyflie
title: Crazyflie
sidebar_label: Platforms
---

This will contain the overview information on how to use the crazyflie with the UdaciDrone API.

Specifically should discuss how to use the CrazyflieConnection class, a short version of what the parameters mean (the long version is in the files themselves if people are very curious), the supported list of commands, anything else specific to note that is related to the crazyflie and its behavior with UdaciDrone.

# UdaciDrone with the Crazyflie #

This document discuses the following:
 - setup of the crazyflie
 - overview of the CrazyflieConnection class
 - platform specific limitations

## Crazyflie Setup ##

TODO: the setup and configuration instructions for the crazyflie

These instructions are for the crazyflie using the Flow Deck.  Note that UdaciDrone will only be able to control a crazyflie that either has the Flow Deck of the LocoPositioning system (these instructions assume a Flow Deck is being used).

TODO: find a better way to word the fact that need the flow deck.

Also will need the CrazyRadio as that will be how we will be communicating between the computer and the crazyflie.

TODO: add a link to the setup instructions for getting the CrazyRadio installed on their device.

The CrazyRadio's default bandwidth is enough to send simple commands, but will not be enough to handle the data used by UdaciDrone without introducing a latency that makes things more difficult.  Therefore the first step, once you have your CrazyRadio installed properly, will be to increase the bandwidth of the data link from the default 250K (meaning 250 kpbs) to 2M (meaning 2 Mbps).  For detailed instructions on how to make this change [check out BitCraze's documentation](https://wiki.bitcraze.io/doc:crazyflie:client:pycfclient:index).  Here is an abridged version:

 1. Open the crazyflie PC client with your crazyradio plugged in to your computer the the crazyflie powered on.

 2. In the `Connect` menu, select the `Configure 2.0` option.  This should display a window that contains a set of trim setting and radio configuration settings.
 
 3. From the dropdown, select the `2Mbit/s` option.  Note if you are flying multiple crazyflies or are having problems with interference, this is also where you change change the radio channel the crazyflie is communicating on.

 4. Save the setting to the crazyflie by hitting `Write`.

TODO: add abridged version

TODO: instructions for configuring the crazyflie to have a bandwidth of 2M and instructions for channel separation if multiple crazyflie's will be used.  For now read through [bitcraze's instructions](https://wiki.bitcraze.io/doc:crazyflie:client:pycfclient:index) which tell you everything, just not the most clear.

A few import notes about the crazyflie:

 - the NED frame used for the crazyflie is not aligned with a world NED frame, rather it is aligned with a body fixed NED frame at takeoff.  This means that any move commands should be seen as:
   -  N = m forward of start position and heading
   -  E = m to the right of start position and heading
   -  D = m height above the ground

 - the crazyflie's height (D) is a value above the ground or whatever is below the crazyflie.  If you have commanded a height of 0.5 and all of a sudden fly over a box that is 0.2 meters tall, the effective flight height of the crazyflie will become 0.7 meters!  Unfortunately the crazyflie doesn't know that the ground level changed, so to it is is simply maintaining 0.5 meters.

 - depending on the ground and battery level of the crazyflie, I have not had the most success with heights >= 1 meter, 0.5 seems to be a really good functional height.

 - On takeoff, the crazyflie will take a second or two before it actually takes off.  This is because it needs to calibrate its estimator onboard and needs to wait for the variance in the position estimate to drop enough.



## CrazyflieConnection ##

The `CrazyflieConnection` take in a URI that represents the "address" for which a connection can be made over.  You can think of this much like an IP address but for a crazyflie.  Your crazyflie URI may be configured slightly differently, but it should match the URI used to connect to your crazyflie [through the crazyflie client](https://www.bitcraze.io/getting-started-with-the-crazyflie-2-0/#connect-pc-client).

**NOTE: for best results, your crazyflie should be configured with a 2M bandwidth**

This connection also optionally supports a default `velocity` parameter, which specifies the flight velocity for the crazyflie.

**NOTE: don't worry too much about the velocity parameter, you'll also be able to change that in flight later!**

```py
conn = CrazyflieConnection('radio://0/80/2M', velocity=0.3)
```

## API Commands ##

Unfortunately the crazyflie is not capable of supporting the full set of commands of the UdaciDrone API, so here is a list of supported commands for your reference:

 - `start()` and `stop()` - start and stop the connection to the drone, respectively
 - `arm()` and `disarm()` - arm and disarm the motors, respectively
 - `disarm()` - disarm the motors
 - `take_control()` and `release_control()` - manage control of the drone from being with the script or with a user
 - `set_home_as_current_position()` - update the world frame home position to be the current position of the drone
 - `takeoff(alt)` - take off to the specified altitude
 - `land()` - land at the current location
 - `cmd_position(n, e, alt, heading)` - command the drone to a specific North, East, Altitude (all in meters) position with the given heading (in radians) **NOTE: for the crazyflie, unfortunately controlling the heading does not work at the moment (causes a lot of error in the position estimate onboard the crazyflie, at least in my testing)**


### Custom Commands ###

There is one crazyflie specific command to change the flight velocity that you can also use that can be *accessed through the connection object itself*:

```py
>>> conn.set_velocity(0.5)  # change the flight velocity to 0.5 m/s
```

**NOTE: this accessed through the connection itself, not through the `Drone` class.**


## Notes ##

This is just a list of notes and behaviors that you should be aware of when using the crazyflie connection.

 - For the crazyflie, the notion and taking control and arming/disarming does not exist!  These functions are used internally to handle some of the state updates for running an event-driven programming script, but that's it.  Therefore, do not expect to see any difference in the crazyflie after sending these commands (unlike in the simulator where the rotors start to turn after the arm command is given).

 - you may not see the crazyflie take off immediately, that is ok!  It needs to reset its position estimator and before taking off it ensures that the variance in position is small enough.  This is done on every takeoff to ensure the best possible position solution.
