"""will contain the connection to a crazyflie drone

[description]
"""

import math
import threading
import time

# crazyflie imports
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

import queue
from udacidrone.connection import message_types as mt
from udacidrone.connection import connection
# udacidrone imports
from udacidrone.messaging import MsgID


class CrazyflieCommand:
    """a very simple class to contain a command that should be sent to the CrazyFlie

    there are a handful of different types of commands that are capable, so
    this wrapper just makes it easier to send them all through the same pipe
    to the thread handling the actual sending of the commands

    Attributes:
        CMD_TYPE_VELOCITY: for sending a velocity command (vx, vy, vz, yawrate)
        CMD_TYPE_HOVER: for sending an altitude hold vel cmd (vx, vy, yawrate, zdist)
        CMD_TYPE_ATTITUDE_THRUST: for sending attitude and thrust (roll, pitch, yaw, thrust)
        CMD_TYPE_ATTITUDE_DIST: for sending attitude and altitude (roll, pitch, yaw, zdist)
        CMD_TYPE_STOP: for telling the crazyflie to stop all motors
    """
    CMD_TYPE_VELOCITY = 1
    CMD_TYPE_HOVER = 2
    CMD_TYPE_ATTITUDE_THRUST = 3
    CMD_TYPE_ATTITUDE_DIST = 4
    CMD_TYPE_STOP = 5

    def __init__(self, cmd_type, cmd, delay=None):
        """create a command

        create all the necessary elements to be able to send a command to the
        crazyflie, using the crazyflie API

        Args:
            cmd_type: the type of command to send (see class enum)
            cmd: the command itself formated as a tuple, (param1, param2, param3, param4)
            delay: the number of SECONDS the command should run (default: {None})
        """
        self.type = cmd_type
        self.cmd = cmd
        self.delay = delay


class CrazyflieConnection(connection.Connection):

    def __init__(self, uri, velocity=0.2, threaded=False):
        super().__init__(threaded)

        # set the default velocity
        self._velocity = velocity

        # Initialize the low-level drivers (don't list the debug drivers)
        cflib.crtp.init_drivers(enable_debug_driver=False)

        # the connection to the crazyflie
        self._scf = SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache'))

        # temp (or maybe will be permanent) state variable
        self._is_open = False
        self._running = False

        self._send_rate = 5  # want to send messages at 5Hz  NOTE: the minimum is 2 Hz
        self._out_msg_queue = queue.Queue()  # a queue for sending data between threads
        self._write_handle = threading.Thread(target=self.command_loop)
        self._write_handle.daemon = True

        # since can only command velocities and not positions, the connection
        # needs some awareness of the current position to be able to do
        # the math necessary
        self._current_position_xyz = [0.0, 0.0, 0.0]  # [x, y, z]

        # state information is to be updated and managed by this connection class
        # for the crazyflie, since the crazyflie doesn't exactly pass down the
        # state information
        #
        # defining the states to be:
        # armed -> should roughly mimic connection state (though this does have a problem at the end...)
        # guided -> this seems to only be used at the end condition.....

        self._armed = True
        self._guided = True

        # kalman filter state
        self._converged = False
        self._var_y_history = [1000] * 10
        self._var_x_history = [1000] * 10
        self._var_z_history = [1000] * 10
        self._filter_threshold = 0.001

    @property
    def open(self):
        """
        Returns:
            Boolean. True if connection is able to send and/or receive messages, False otherwise.
        """
        # TODO: figure out the open condition for crazyflie
        if self._is_open == -1:
            return False
        return True

    def start(self):
        """Command to start a connection with a drone"""
        self._scf.open_link()  # this is a blocking function that will not return until the link is opened

        # TODO: need a better version of this
        self._is_open = True

        # need to now register for callbacks on the data of interest from the crazyflie
        # TODO: decide on the appropriate rates
        log_pos = LogConfig(name='LocalPosition', period_in_ms=500)
        log_pos.add_variable('kalman.stateX', 'float')
        log_pos.add_variable('kalman.stateY', 'float')
        log_pos.add_variable('kalman.stateZ', 'float')
        try:
            self._scf.cf.log.add_config(log_pos)
            # This callback will receive the data
            log_pos.data_received_cb.add_callback(self._cf_callback_pos)
            # This callback will be called on errors
            log_pos.error_cb.add_callback(self._cf_callback_error)
            # Start the logging
            log_pos.start()
        except KeyError as e:
            print('Could not start position log configuration,' '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Position log config, bad configuration.')

        log_vel = LogConfig(name='LocalVelocity', period_in_ms=500)
        log_vel.add_variable('kalman.statePX', 'float')
        log_vel.add_variable('kalman.statePY', 'float')
        log_vel.add_variable('kalman.statePZ', 'float')
        try:
            self._scf.cf.log.add_config(log_vel)
            # This callback will receive the data
            log_vel.data_received_cb.add_callback(self._cf_callback_vel)
            # This callback will be called on errors
            log_vel.error_cb.add_callback(self._cf_callback_error)
            # Start the logging
            log_vel.start()
        except KeyError as e:
            print('Could not start velocity log configuration,' '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add velocity log config, bad configuration.')

        log_att = LogConfig(name='Attitude', period_in_ms=500)
        log_att.add_variable('stabilizer.roll', 'float')
        log_att.add_variable('stabilizer.pitch', 'float')
        log_att.add_variable('stabilizer.yaw', 'float')
        try:
            self._scf.cf.log.add_config(log_att)
            # This callback will receive the data
            log_att.data_received_cb.add_callback(self._cf_callback_att)
            # This callback will be called on errors
            log_att.error_cb.add_callback(self._cf_callback_error)
            # Start the logging
            log_att.start()
        except KeyError as e:
            print('Could not start attitude log configuration,' '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add attitude log config, bad configuration.')

        log_state = LogConfig(name='State', period_in_ms=1000)
        log_state.add_variable('kalman.inFlight', 'uint8_t')  # TODO: check the data type
        try:
            self._scf.cf.log.add_config(log_state)

            log_state.data_received_cb.add_callback(self._cf_callback_state)
            log_state.error_cb.add_callback(self._cf_callback_error)

            # Start the logging
            log_state.start()
        except KeyError as e:
            print('Could not start position log configuration,' '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add state log config, bad configuration.')


        # start the write thread now that the connection is open
        self._running = True
        self._write_handle.start()

    def stop(self):
        """Command to stop a connection with a drone"""

        # need to send a stop command
        cmd = CrazyflieCommand(CrazyflieCommand.CMD_TYPE_STOP, None)
        self._out_msg_queue.put(cmd)
        time.sleep(0.5)  # add a sleep to make sure this command is sent and executed properly
        self._running = False  # this is to stop the command thread
        time.sleep(1)  # make sure the command thread has stopped

        self._scf.close_link()  # close the link

        # TODO: find a better way to handle this...
        self._is_open = False

    def dispatch_loop(self):
        """Main loop that triggers callbacks as messages are recevied"""
        # NOTE: nothing needed here for the crazyflie
        # NOTE: the crazyflie API sends data down as callbacks already, so this
        # NOTE: connection class is nothing but a passthrough for those
        pass

    def command_loop(self):
        """loop to send commands at a specified rate"""

        # the last time a command was sent
        # to be used to ensure commands are at the desired rate
        last_write_time = time.time()

        cmd_start_time = 0  # the time [s] that the command started -> needed for distance commands

        # the current command that should be being sent, default to 0 everything
        current_cmd = CrazyflieCommand(CrazyflieCommand.CMD_TYPE_STOP, None)

        # the last commanded height
        # if this is not 0, want the hold commands to be hover to hold the specific height
        current_height = 0

        while self._running:

            # want to make sure the kalman filter has converged before sending any command
            while not self._converged:
                continue

            # empty out the queue of pending messages -> want to always send the messages asap
            # NOTE: this effectively only sends the last command in the queue....
            # TODO: see if need to handle the fact that maybe want to send all the commands in the queue...
            cmd = None
            new_cmd = False
            while not self._out_msg_queue.empty():
                try:
                    cmd = self._out_msg_queue.get_nowait()
                except queue.Empty:
                    # if there is no msgs in the queue, will just continue
                    pass
                else:
                    if cmd is not None:
                        new_cmd = True
                        current_cmd = cmd
                        cmd_start_time = time.time()
                        # TODO: maybe want to handle the command here...
                        self._out_msg_queue.task_done()

                        # DEBUG
                        # print("recevied command, type {}, cmd {}, delay {}".format(
                        # current_cmd.type, current_cmd.cmd, current_cmd.delay))

            # first thing to check is the timer, if applicable
            if current_cmd.delay is not None:
                if time.time() - cmd_start_time >= current_cmd.delay:
                    # DEBUG
                    # print("command timer completed, completed command: {}, {}".format(
                    # current_cmd.type, current_cmd.cmd))

                    # time to stop
                    new_cmd = True
                    if current_height > 0:
                        current_cmd = CrazyflieCommand(CrazyflieCommand.CMD_TYPE_HOVER, (0.0, 0.0, 0.0, current_height))
                    else:
                        current_cmd = CrazyflieCommand(CrazyflieCommand.CMD_TYPE_VELOCITY, (0.0, 0.0, 0.0, 0.0))

            # if this isn't a new command, want to rate limit accordingly
            # rate limit the loop
            if not new_cmd:
                current_time = time.time()
                if (current_time - last_write_time) < (1.0 / self._send_rate):
                    continue
                last_write_time = time.time()

                # DEBUG
                # print("sending command, type {}, cmd {}, delay {}".format(
                # current_cmd.type, current_cmd.cmd, current_cmd.delay))

            # TODO: probably need to constantly update the velocity commands here....
            # TODO: effectively need to wrap a high level control loop, at the very least on height

            # now do the actual sending of the command
            if current_cmd.type == CrazyflieCommand.CMD_TYPE_VELOCITY:
                self._scf.cf.commander.send_velocity_world_setpoint(*current_cmd.cmd)

            elif current_cmd.type == CrazyflieCommand.CMD_TYPE_HOVER:
                current_height = current_cmd.cmd[3]
                self._scf.cf.commander.send_hover_setpoint(*current_cmd.cmd)

            elif current_cmd.type == CrazyflieCommand.CMD_TYPE_ATTITUDE_THRUST:
                self._scf.cf.commander.send_setpoint(*current_cmd.cmd)

            elif current_cmd.type == CrazyflieCommand.CMD_TYPE_ATTITUDE_DIST:
                current_height = current_cmd.cmd[3]
                self._scf.cf.commander.send_zdistance_setpoint(*current_cmd.cmd)

            elif current_cmd.type == CrazyflieCommand.CMD_TYPE_STOP:
                # TODO: probably want to send appropriate flags that state disarmed, etc
                # TODO: basically need to update the drone state here
                self._scf.cf.commander.send_stop_setpoint()

            else:
                print("invalid command type!")

    def _cf_callback_pos(self, timestamp, data, logconf):
        x = data['kalman.stateX']
        y = data['kalman.stateY']
        z = data['kalman.stateZ']
        # print("current height: {}".format(z))
        self._current_position_xyz = [x, y, z]  # save for our internal use
        pos = mt.LocalFrameMessage(timestamp, x, -y, -z)
        self.notify_message_listeners(MsgID.LOCAL_POSITION, pos)

    def _cf_callback_vel(self, timestamp, data, logconf):
        x = data['kalman.statePX']
        y = data['kalman.statePY']
        z = data['kalman.statePZ']
        vel = mt.LocalFrameMessage(timestamp, x, y, z)
        self.notify_message_listeners(MsgID.LOCAL_VELOCITY, vel)

    def _cf_callback_att(self, timestamp, data, logconf):
        roll = data['stabilizer.roll']
        pitch = data['stabilizer.pitch']
        yaw = data['stabilizer.yaw']
        fm = mt.FrameMessage(timestamp, roll, pitch, yaw)
        self.notify_message_listeners(MsgID.ATTITUDE, fm)

    def _cf_callback_state(self, timestamp, data, logconf):
        # in_flight = data['kalman.inFlight']
        # armed = False
        # guided = False
        # if in_flight:
        #     armed = True
        #     guided = True

        # send a state message to the drone to set armed and guided to be True
        # since these constructs don't exist for the crazyflie, but the armed -> guided transition needs to be 
        # robust to work from the sim to the crazyflie with minimal changes
        state = mt.StateMessage(timestamp, self._armed, self._guided)
        self.notify_message_listeners(MsgID.STATE, state)

        # TODO: probably need a better metric for armed / guided
        # since the quad is basically always armed and guided comes into play
        # once the connection is made, so basically the second the script starts...
        # state = mt.StateMessage(timestamp, armed, guided)
        # self.notify_message_listeners(MsgID.STATE, state)
        pass

    def _cf_callback_kf_variance(self, timestamp, data, logconf):
        self._var_x_history.append(data['kalman.varPX'])
        self._var_x_history.pop(0)
        self._var_y_history.append(data['kalman.varPY'])
        self._var_y_history.pop(0)
        self._var_z_history.append(data['kalman.varPZ'])
        self._var_z_history.pop(0)

        min_x = min(self._var_x_history)
        max_x = max(self._var_x_history)
        min_y = min(self._var_y_history)
        max_y = max(self._var_y_history)
        min_z = min(self._var_z_history)
        max_z = max(self._var_z_history)

        # print("filter variances: {} {} {}".format(max_x - min_x, max_y - min_y, max_z - min_z))

        if (max_x - min_x) < self._filter_threshold and (max_y - min_y) < self._filter_threshold and (max_z - min_z) < self._filter_threshold:
            print("filter has converge, position is good!")
            self._converged = True
            self._kf_log_config.stop()  # no longer care to keep getting the kalman filter variance

    def _cf_callback_error(self, logconf, msg):
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _wait_for_position_estimator(self):
        """Start listening for the kalman filter variance to determine when it converges"""
        print('Waiting for estimator to find position...')

        # configure the log for the variance
        self._kf_log_config = LogConfig(name='Kalman Variance', period_in_ms=100)
        self._kf_log_config.add_variable('kalman.varPX', 'float')
        self._kf_log_config.add_variable('kalman.varPY', 'float')
        self._kf_log_config.add_variable('kalman.varPZ', 'float')

        try:
            self._scf.cf.log.add_config(self._kf_log_config)
            # This callback will receive the data
            self._kf_log_config.data_received_cb.add_callback(self._cf_callback_kf_variance)
            # This callback will be called on errors
            self._kf_log_config.error_cb.add_callback(self._cf_callback_error)
            # Start the logging
            self._kf_log_config.start()
        except KeyError as e:
            print('Could not start kalman log configuration,' '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add kalman log config, bad configuration.')

    def _reset_position_estimator(self):
        """reset the estimator to give the best performance possible"""
        self._scf.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self._scf.cf.param.set_value('kalman.resetEstimation', '0')
        # TODO: instead of a sleep, probably want a condition on the variance
        # time.sleep(2)
        self._wait_for_position_estimator()

    def set_velocity(self, velocity):
        """set the velocity the drone should use in flight"""
        self._velocity = velocity

    def arm(self):
        """Command to arm the drone"""
        # NOTE: this doesn't exist for the crazyflie
        # TODO: could have arm or take control be where we reset the estimator...
        pass

    def disarm(self):
        """Command to disarm the drone"""
        # NOTE: this doesn't exist for the crazyflie
        pass

    def take_control(self):
        """
        Command the drone to switch into a mode that allows external control.
        e.g. for PX4 this commands 'offboard' mode, while for APM this commands 'guided' mode
        """
        # NOTE: this doesn't exist for the crazyflie
        # however, if this command is being used, want to make sure the state output conforms to the expected changes
        self._armed = True
        self._guided = True
        pass

    def release_control(self):
        """Command to return the drone to a manual mode"""
        # NOTE: this doesn't exist for the crazyflie
        # however, if this command is being used, want to make sure the state output conforms to the expected changes
        self._armed = False
        self._guided = False
        pass

    def cmd_attitude(self, roll, pitch, yawrate, thrust):
        """Command to set the desired attitude and thrust

        Args:
            yaw: the desired yaw in radians
            pitch: the desired pitch in radians
            roll: the deisred roll in radians
            thrust: the normalized desired thrust level on [0, 1]
        """
        pass

    def cmd_attitude_rate(self, roll_rate, pitch_rate, yaw_rate, thrust):
        """Command to set the desired attitude rates and thrust

        Args:
            yaw_rate: the desired yaw rate in radians/second
            pitch_rate: the desired pitch rate in radians/second
            roll_rate: the desired roll rate in radians/second
            thrust: the normalized desired thrust level on [0, 1]
        """
        pass

    def cmd_moment(self, roll_moment, pitch_moment, yaw_moment, thrust):
        """Command to set the desired moments and thrust

        Args:
            roll_moment: the desired roll moment in Newton*meter
            yaw_moment: the desired yaw moment in Newton*meter
            pitch_moment: the desired pitch moment in Newton*meter
            thrust: the normalized desired thrust level in Newton
        """
        pass

    def cmd_velocity(self, vn, ve, vd, heading):
        """Command to set the desired velocity (NED frame) and heading

        Args:
            vn: desired north velocity component in meters/second
            ve: desired east velocity component in meters/second
            vd: desired down velocity component in meters/second (note: positive down!)
            heading: desired drone heading in radians
        """
        pass

    def cmd_motors(self, motor1, motor2, motor3, motor4):
        """Command the thrust levels for each motor on a quadcopter

        Args:
            motor1: normalized thrust level for motor 1 on [0, 1]
            motor2: normalized thrust level for motor 2 on [0, 1]
            motor3: normalized thrust level for motor 3 on [0, 1]
            motor4: normalized thrust level for motor 4 on [0, 1]
        """
        pass

    def cmd_position(self, n, e, d, heading):
        """Command to set the desired position ("NED" frame) and heading

        Note: For the crazyflie, NED is really the body XYZ frame fixed 
        unpon startup of the crazyflie to be a world frame

        Args:
            n: desired north position in meters
            e: desired east position in meters
            d: desired down position in meters (note: positive down!)
            heading: desired drone heading in radians
        """

        # need to know the current position: for now going to simply map NED to XYZ!!!
        # x is forward
        # y is left
        # z is up
        # also completely ignoring heading for now

        # DEBUG
        # print("current position (x,y,z): ({}, {}, {})".format(
        # self._current_position_xyz[0], self._current_position_xyz[1], self._current_position_xyz[2]))
        # print("commanded position (x,y,z): ({}, {}, {})".format(n, -e, -d))

        # calculate the change vector needed
        # note the slight oddity that happens in converting NED to XYZ
        # as things are used as XYZ internally for the crazyflie
        dx = n - self._current_position_xyz[0]
        dy = -e - self._current_position_xyz[1]
        z = -1 * d  # holding a specific altitude, so just pass altitude through directly

        # DEBUG
        # print("move vector: ({}, {}) at height {}".format(dx, dy, z))

        distance = math.sqrt(dx * dx + dy * dy)
        delay_time = distance / self._velocity

        # DEBUG
        # print("the delay time for the move command: {}".format(delay_time))

        # need to now calculate the velocity vector -> need to have a magnitude of default velocity
        vx = self._velocity * dx / distance
        vy = self._velocity * dy / distance

        # DEBUG
        # print("vel vector: ({}, {})".format(vx, vy))

        # create and send the command
        # TODO: determine if would want to use the hover command instead of the velocity command....
        # TODO: problem with the hover command is have no feedback on the current altitude!!
        cmd = CrazyflieCommand(CrazyflieCommand.CMD_TYPE_HOVER, (vx, vy, 0.0, z), delay_time)
        self._out_msg_queue.put(cmd)

    def cmd_relative_position(self, dx, dy, z, heading):
        print("move vector: ({}, {}) at height {}".format(dx, dy, z))

        distance = math.sqrt(dx * dx + dy * dy)
        delay_time = distance / self._velocity
        print("the delay time for the move command: {}".format(delay_time))

        # need to now calculate the velocity vector -> need to have a magnitude of default velocity
        vx = self._velocity * dx / distance
        vy = self._velocity * dy / distance
        print("vel vector: ({}, {})".format(vx, vy))

        # create and send the command
        # TODO: determine if would want to use the hover command instead of the velocity command....
        # TODO: problem with the hover command is have no feedback on the current altitude!!
        cmd = CrazyflieCommand(CrazyflieCommand.CMD_TYPE_HOVER, (vx, vy, 0.0, z), delay_time)
        self._out_msg_queue.put(cmd)

    def takeoff(self, n, e, d):
        """Command the drone to takeoff.

        Note some autopilots need a full position for takeoff
        and since this class is not aware of current position.`n` and `e`
        must be passed along with `d` for this command.

        Args:
            n: current north position in meters
            e: current east position in meters
            altitde: desired altitude
        """
        # first step: reset the estimator to make sure all is good, this will take a variable amount of time
        # as it waits for the filter to converge before returning
        self._reset_position_estimator()

        # add to queue a command with 0 x,y vel, 0 yawrate, and the desired height off the ground
        cmd = CrazyflieCommand(CrazyflieCommand.CMD_TYPE_HOVER, (0.0, 0.0, 0.0, d))
        self._out_msg_queue.put(cmd)

    def land(self, n, e):
        """Command the drone to land.

        Note some autopilots need a full position for landing
        and since this class is not aware of current position.`n` and `e`
        must be passed along with `d` for this command.

        Args:
            n: current north position in meters
            e: current east position in meters
        """
        # need to know the current height here...
        current_height = self._current_position_xyz[2]
        decent_velocity = -self._velocity/2  # [m/s]

        # calculate how long that command should be executed for
        # we aren't going to go all the way down before then sending a stop command
        # TODO: figure out a way to do this without sleeping!!
        delay_time = (current_height - 0.02) / (-1 * decent_velocity)  # the wait time in seconds

        # DEBUG
        # print("current height: {}, delay time: {}".format(current_height, delay_time));

        cmd = CrazyflieCommand(CrazyflieCommand.CMD_TYPE_VELOCITY, (0.0, 0.0, decent_velocity, 0.0), delay_time)
        self._out_msg_queue.put(cmd)

        # wait the desired amount of time and then send a stop command to kill the motors
        time.sleep(delay_time)
        self._out_msg_queue.put(CrazyflieCommand(CrazyflieCommand.CMD_TYPE_STOP, None))

    def set_home_position(self, lat, lon, alt):
        """Command to change the home position of the drone.

        Args:
            lat: desired home latitude in decimal degrees
            lon: desired home longitude in decimal degrees
            alt: desired home altitude in meters (AMSL)
        """
        # NOTE: this concept doesn't exist for the crazyflie
        pass
