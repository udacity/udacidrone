---
id: getting-started
title: Getting Started
sidebar_label: Getting Started
---

TODO: this should be some brief code examples for how one would actually use this API.

To be able to demonstrate how the UdaciDrone API works, we'll need ourselves a fake drone.  To get this part, [download Udacity's drone simulator]() so you can run some of the example code below!

## Requirements ##

This API requires Python 3.

## Installing UdaciDrone ##

First things first, let's install UdaciDrone!

```sh
pip install udacidrone
```

## Using UdaciDrone ##

There are two main parts to the UdaciDrone API, the `Drone` class and different types of connection classes.

```python
>>> from udacidrone import Drone
>>> from udacidrone.connection import MavlinkConnection
```

These are the imports that will let you get started working with a drone that is connected using the mavlink communication protocol.


## Setting Up a Connection ##

TODO: some example code for how to set up a mavlink connection 

```python
>>> conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=True)
```

Note: These examples will be commanding the drone through a python terminal and therefore our connection class takes in an additional parameter of `threaded=True`.  If you are not running this in a python terminal (e.g. a simply script), that parameter is not needed.  For more detail on what this parameter does [check out the connection api documentation](connection.md)

## Setting Up the Drone ##

TODO: some example code for how to initialize a drone

```python
>>> drone = Drone(conn)
```

Once we have a connection set up, it's really easy to set up our `Drone`!  We just need to tell it over which connection it is receiving information and it is ready to go.

In order to start controlling it, etc, we do need to start the connection between our code and the drone itself:

```python
>>> drone.start()
```


## Example ##

So now we have something that looks a little like this:

```python
>>> from udacidrone import Drone
>>> from udacidrone.connection import MavlinkConnection
>>> conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=True)
>>> drone = Drone(conn)
>>> drone.start()
```

So now what can we do?

Well we would love to be able to takeoff, however before we can take off, we first much get the drone ready for takeoff.  This is a process that requires both taking control of the drone and arming it for takeoff.

```python
>>> drone.take_control()
>>> drone.arm()
```

Now we are ready to send some more fun commands, for example if we run:

```python
>>> drone.takeoff(3)
```

you should see the drone takeoff to 3 meters above the ground in the simulator!

To see all the other commands you have available to you and to further explore the Drone API [check out the Drone API's detailed documentation](drone.md)!
