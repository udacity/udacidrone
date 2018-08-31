---
id: tutorial-shell
title: UdaciDrone in the Python Shell
sidebar_label:  UdaciDrone in the Python Shell
---

This is a tutorial on the use of UdaciDrone to interact with a drone through the Python shell (using Python 3).  Note that this is really platform agnostic! (Or Connection class agnostic to be specific with the UdaciDrone terms).

The following example can be used for controlling the simulator, a crazyflie or even a PX4 drone!  The only changes for each of these will be the instance of the connection that is passed to the `Drone`.  When working in the python shell, commands can be sent on at a time and the behavior of the drone can immediately be seen.

To learn more about the respective connections, check out the individual documentation.


## Imports ##

Before being able to use the UdaciDrone API, you will need to import it into the python shell.  In the shell, we need to import both the `Drone` class (which we will be using to interact with the drone) and the connection class to be used to connect to the drone.  For this example, we will be assuming we are connecting to the simulator and will be using the `MavlinkConnection`.

*For more information on the use of the difference connection classes, check out [the connection tutorial]()*.

```py
>>> from udacidrone import Drone
>>> from udacidrone.connection import MavlinkConnection
```

## Connections ##

The first step will be to create the connection class that will be used by `Drone` to communicate to your drone (either real or simulated).  For this tutorial, we will be continuing with our example using a connection to the simulator.  If you are controlling a different device, make sure to use the appropriate connection constructor here.  For more details on these options check out the [connection class detailed documentation]().

```py
>>> conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=True, PX4=False)
```

## Drone ##

Now that we have a connection, we can create our `Drone` which takes in an instance of a `Connection` class to be able to communicate with the drone itself.  Once created, we can call `start()` to start the connection and get flying!

**NOTE: at the end of your flight, make sure you call `stop()` to properly close the connection to the drone**

```py
>>> drone = Drone(conn)
>>> drone.start()
```

### Arming / Disarming ###

Before flying, you will need to signal to the drone that you will be controlling it from your computer and that you want to arm it for flight.  To do this, we have 2 separate commands that need to be executed:

```py
>>> drone.take_control()  # signals to the drone to accept commands from the computer
>>> drone.arm()  # signals to the drone to be ready for takeoff
```

**NOTE: at the end of your flight you will need to do the reverse process:**

```py
>>> drone.disarm()
>>> drone.release_control()
```

*NOTE: not all drones will exhibit a different behavior once these commands have been sent!  For platform specific behavior, make sure to look at the details for that specific platform in the documentation.*

### Flying ###

Before taking off, it is safest to reset the home position to the current position!  This makes it so that the drone's current position in our coordinate frame is (0,0,0), making it easier for you to send position commands.

```py
>>> drone.set_home_as_current_position()
```

Once the home position is set, you are ready to take off and start flying around.  For example, try out these commands one at a time:

```py
>>> drone.takeoff(0.5)  # take off to 0.5m above the ground
>>> drone.cmd_position(2, 1, 0.5, 0)  # fly to a point 2 meters north, 1 meter east, at an altitude of 0.5 meters with a heading of 0 rads
>>> drone.cmd_position(0, 0, 0.5, 0)
>>> drone.land()
```

### Commands ###

TODO: link to somewhere else for the full list of supported commands per platform.  While the API has a common set of commands, unfortunately not every platform is able to support every type of command.
