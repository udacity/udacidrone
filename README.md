
## Drone API

This is the Udacity Drone API. It provides an interface for communicating with your quadcopter in the simulators provided in the Flying Car Nanodegree Program, and for communicating with a real drone if you wish to do so.

To use this package, you'll need to first install the [Python Starter Kit](https://github.com/udacity/FCND-Term1-Starter-Kit) provided for this Nanodegree Program.  

### The Code
Within `drone.py` we wrote a wrapper `Drone` superclass to handle all the communication between Python and the drone simulator. The `Drone` class contains commands to be passed to the simulator and allows you to register callbacks/listeners on messages coming from the simulator. 

### Incoming Message Types

The following incoming message types are available for use with the Drone API:

* `state_msg`: Information about whether the drone is armed and in guided mode
* `global_position_msg`: latitude, longitude, altitude
* `local_position_msg`: local north, local east, local down
* `local_velocity_msg`: local north velocity, local east velocity, local vertical velocity (positive up)

All message types also contain the time. More information about the properties of each message can be found in `message_types.py`. The data for these messages are retrieved using callbacks.

### Registering Callbacks

The incoming message data is receiving using callback methods. These methods are only called when a message of their respective type is received.  Callbacks are member functions of the drone class and are registered/unregistered using `self.register_callback(msg_type, callback_fn)` and `self.remove_callback(msg_type, callback_fn)`, respectively.

To register a callback for a particular message type (`MSG_GLOBAL_POSITION` in this case): 

```python
self.register_callback(message_types.MSG_GLOBAL_POSITION, 								self.global_position_callback)

def global_position_callback(self):
	# do whatever you want knowing that the drone global position attribute has new information
```

Callbacks are triggered when the corresponding attributes of the drone class have been updated with new information from the simulator or real drone.  For this reason, you will notice that the callbacks do not require any input variables (besides the `self` parameter required for a class member function).

In some cases, you may find it useful to have a single callback for all attribute changes, which can be done by passing `MsgId.ANY` as the message type.  For this callback, an additional parameter of the name of the attribute set that has been updated is also passed to the callback.

```python
self.register_callback('*', self.all_msg_callback)
def all_msg_callback(self, name):
	# this is a listener for all message types, so break out the msg as defined by the name
```
        



### Drone Attributes

In addition to being passed to appropriate callbacks, the message data is also saved into the following attributes of the `Drone` class:

* `global_position`: latitude (deg), longitude (deg), altitude (meter)
* `local_position`: north (meter), east (meter), down (meter)
* `local_velocity`: north velocity (m/s), east velocity (m/s), vertical velocity (m/s, positive up)
* `armed`: True/False
* `guided`: True/False

Drone attributes can be used if information is required from multiple messages. For example:

```python
def global_position_callback(self):
	if self.global_position[2] < 0.05: # Checks the global altitude
        if self.local_velocity[2] < 0.05 # Checks the latest drone velocity, since it isn't part of the message
```


### Outgoing Commands

The following commands are implemented within the `Drone` superclass:

* `connect()`: Starts receiving messages from the drone. Blocks the code until the first message is received
* `start()`: Start receiving messages from the drone. If the connection is not threaded, this will block the code.
* `arm()`: Arms the motors of the quad, the rotors will spin slowly. The drone cannot takeoff until armed first
* `disarm()`: Disarms the motors of the quad. The quadcopter cannot be disarmed in the air
* `take_control()`: Set the command mode of the quad to guided
* `release_control()`: Set the command mode of the quad to manual
* `cmd_position(north, east, down, heading)`: Command the drone to travel to the local position (north, east, down). Also commands the quad to maintain a specified heading
* `takeoff(target_altitude)`: Takeoff from the current location to the specified global altitude
* `land()`: Land in the current position
* `stop()`: Terminate the connection with the drone and close the telemetry log

These can be called directly from other methods within the `Drone` class, i.e.:

```python
self.arm() # Sends an arm command to the drone
```

### Manual Flight Using the Drone API

To log data while flying manually, run the `drone.py` script as shown below:

```sh
python drone.py
```

Run this script after starting the simulator. It connects to the simulator using the `Drone` class and runs until the tcp connection is broken. The connection will timeout if it doesn't receive a heartbeat message once every 10 seconds. The GPS data is automatically logged.

To stop logging data, stop the simulator first and the script will automatically terminate after approximately 10 seconds.

Alternatively, the drone can be manually started/stopped from a Python/iPython shell:

```python
from drone import Drone
drone = Drone()
drone.start(threaded=True, tlog_name="TLog-manual.txt")
```
When starting the drone manually from a Python/iPython shell you have the option to provide a desired filename for the telemetry log file (such as "TLog-manual.txt" as shown above).  This allows you to customize the telemetry log name as desired to help keep track of different types of log files you might have.  Note that when running the drone from `python drone.py` for manual flight, the telemetry log will default to "TLog-manual.txt".

If `threaded` is set to `False`, the code will block and the drone logging can only be stopped by terminating the simulation. If the connection is threaded, the drone can be commanded using the commands described above, and the connection can be stopped (and the log properly closed) using:

```python
drone.stop()
```

### Message Logging

The telemetry data is automatically logged in "Logs\TLog.txt" or "Logs\TLog-manual.txt" for logs created when running `python drone.py`. Each row contains a comma seperated representation of each message. The first row is the incoming message type. The second row is the time. The rest of the rows contain all the message properties. 

#### Reading Telemetry Logs

Logs can be read using:

```python
t_log = drone.read_telemetry_data(filename)
```

The data is stored as a dictionary of message types. For each message type, there is a list of numpy arrays. For example, to access the longitude and latitude from a `global_position_msg`:

```python
# Time is always the first entry in the list
time = t_log['global_position_msg'][0][:]
longitude = t_log['global_position_msg'][1][:]
latitude = t_log['global_position_msg'][2][:]
```

The data between different messages will not be time synced since they are recorded at different times.


## Autonomous Control State Machine

After getting familiar with how the drone flies, you will fill in the missing pieces of a state machine to fly the drone autonomously. The state machine is run continuously until either the mission is ended or the MAVLink connection is lost.

The six states predefined for the state machine:  

* `MANUAL`: the drone is being controlled by the user
* `ARMING`: the drone is in guided mode and being armed
* `TAKEOFF`: the drone is taking off from the ground
* `WAYPOINT`: the drone is flying to a specified target position
* `LANDING`: the drone is landing on the ground
* `DISARMING`: the drone is disarming

While the drone is in each state, you will need to check transition criteria with a registered callback. If the transition criteria are met, you will set the next state and pass along any commands to the drone. For example:

```python
def state_callback():
	if self.state == States.DISARMING:
    	if ~self.armed:
        	self.release_control()
        	self.in_mission = False
        	self.state = States.MANUAL
```
This is an example of a callback on the state message. It only checks criteria if it's in the `DISARMING` state. If it detects that the drone is successfully disarmed, it sets the mode back to manual and terminates the mission. More information about which callbacks you need to define will be provided in the context of the projects.      


### Reference Frames

Two different reference frames are defined and used within the Drone API. Global positions are defined as [longitude, latitude, altitude (positive up)]. Local reference frames are defined [North, East, Down (positive up)] and is relative to a nearby global home provided. Both reference frames are defined in a proper right-handed reference frame. The global reference frame is what is provided by the Drone's GPS. Two convenience functions, `global_to_local()` and `local_to_global()` are provided within the `frame_utils.py` script to convert between the two frames. These functions are wrappers on `utm` library functions.


