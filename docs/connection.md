---
id: connection-api
title: Connection API Overview
sidebar_label: Overview
---

TODO: this should be a brief overview of how the connection classes are structures.

Basically we have a `Connection` class that is really just an abstract class (depending on your background, you could also just think of it as an interface).

For each communication protocol our API wants to support, there needs to be a protocol specific class that subclasses the `Connection` class and implements all of the required functions in that protocol.

So really, when we say protocol independent, it's the `Drone` facing side of things that is protocol independent.  All the `Drone` attributes and functions that you would interact with when using UdaciDrone are protocol independent.  We achieve this by having our own set of attributes, etc, so each protocol needs to correctly parse messages and send information to the drone accordingly.

For example, with the mavlink protocol, things like global position and velocity are in the same message, but we have it broken out as two different attributes (and effectively two different internal messages -> from the connection class to the drone).


Currently only the Mavlink protocol is supported, but for those interested in extending the UdaciDrone API to support other protocols (e.g. Parrot or DJI), please refer to [this documentation](extending-connection.md) to see how to get started!

TODO: need to explain all of the input parameters, specifically explain when to use threaded and not threaded!!