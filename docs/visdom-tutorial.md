---
id: visdom-tutorial
title: Plotting Streams Of Data
sidebar_label: Plotting Streams Of Data
---

While the simulator will provide plotting functionality out of the box, you wish to customize it, or you may be working with actual hardware. This tutorial will show you how to plot data in as it streams in using [`visdom`](https://github.com/facebookresearch/visdom/). The complete code can be viewed [here](https://github.com/udacity/udacidrone/blob/master/examples/plot_data.py).

## Code

explain redrawing data using callbacks

## Demo

Prior to plotting anything we have to start the `visdom` server:

![Start visdom server](../assets/start-visdom.gif)

With everything in order, the end result should look something like this:

![Plot streaming data](../assets/plot-visdom2.gif)

<!-- ![Plot streaming data](../assets/plot-visdom.webm) -->


