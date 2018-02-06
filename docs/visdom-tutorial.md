---
id: visdom-tutorial
title: Plotting Realtime Data
sidebar_label: Plotting Realtime Data
---

While the FCND simulator provides plotting functionality out of the box, you wish to customize it, or be working with actual hardware. This tutorial will show you how extend the `Drone` class to plot data in as it streams in using [`visdom`](https://github.com/facebookresearch/visdom/). 

## Start Visdom Server

Prior to plotting anything we have to start the `visdom` server:

![Start visdom server](../assets/visdom-tutorial/server.gif)

## Code

`visdom` has several built-in [plot types](https://github.com/facebookresearch/visdom/#plotting). We'll plot the NED position by using a scatter plot for NE and a line plot for D. For now let's focus on plotting the initial NED position.

### Initial NED Position

The NED position can be accessed with the `local_position` method of the `Drone` class which returns a numpy array of the form `[N, E, D]`.

```python
class MyDrone(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        # default opens up to http://localhost:8097
        self.v = visdom.Visdom()
        assert self.v.check_connection()

		# Plot NE
		ne = np.array(self.local_position[0], self.local_position[1]).reshape(-1, 2)
		self.ne_lot = self.v.scatter(ne, opts=dict(
			title="Local position (north, east)", 
			xlabel='North', 
			ylabel='East'
		))

		# Plot D
		d = np.array(self.local_position[2])
		self.t = 1
		self.d_plot = self.v.line(d, X=np.array(self.t), opts=dict(
			title="Altitude (meters)", 
			xlabel='Timestep', 
			ylabel='Down'
		))

```

![Initial Point](../assets/visdom-tutorial/initial-point.gif)

Awesome, we've got plots up and running! Now, about the realtime thing ...

### Update The Plots

Updating the plots isn't too tricky, and can be with callbacks. We'll define two callbacks `update_ne_plot` and `update_d_plot` that are called whenever a `LOCAL_POSITION` message is received and register them with `register_callback`:

```python
	# code abbreviated

	self.register_callback(MsgID.LOCAL_POSITION, self.update_ne_plot)
	self.register_callback(MsgID.LOCAL_POSITION, self.update_d_plot)

    def update_ne_plot(self):
        ne = np.array([self.local_position[0], self.local_position[1]]).reshape(-1, 2)
        self.v.scatter(ne, win=self.ne_plot, update='append')

    def update_d_plot(self):
        d = np.array([self.local_position[2]])
		# update timestep
        self.t += 1
        self.v.line(d, X=np.array([self.t]), win=self.d_plot, update='append')
```

In `update_ne_plot` and `update_d_plot` we're updating the plot based on the current NED position. `self.v.scatter(ne, win=self.ne_plot, update='append')` notifies `visdom` to update the `self.ne_plot` with `ne` by appending the new data point. Changing this to `update='replace'` will change the update behaviour to deleting the previous data points prior to drawing the new data point.

The end result should look something like this:

<video width="100%" height="540" controls autoplay loop>
<source src="../assets/visdom-tutorial/realtime.webm" type="video/webm">
</video>


### Bonus Exercise: Tracking Data

While the current solution works great it may be keeping more data than we'd like. Multiple graphs ingesting data at high Hz rates could easily get out of hand. In this exercise, should you choose to accept it, you'll be tasked with
keeping track of only the 100 most recent data points.


