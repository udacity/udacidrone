"""will contain the connection to a crazyflie drone

[description]
"""

import math
import queue
import threading
import time

# crazyflie imports
import cflib.crtp
import numpy as np
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from udacidrone.connection import connection
from udacidrone.connection import message_types as mt
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
        CMD_TYPE_POSITION: eventually for sending an alt hold vel cmd,
                           but the command is only a position that needs to be
                           converted into the cf frame for velocity calculation
    """
    CMD_TYPE_VELOCITY = 1
    CMD_TYPE_HOVER = 2
    CMD_TYPE_ATTITUDE_THRUST = 3
    CMD_TYPE_ATTITUDE_DIST = 4
    CMD_TYPE_STOP = 5

    CMD_TYPE_POSITION = 6

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

        self._send_rate = 50  # want to send messages at 5Hz  NOTE: the minimum is 2 Hz
        self._out_msg_queue = queue.Queue()  # a queue for sending data between threads
        self._write_handle = threading.Thread(target=self.command_loop)
        self._write_handle.daemon = True

        # since can only command velocities and not positions, the connection
        # needs some awareness of the current position to be able to do
        # the math necessary
        self._current_position_xyz = np.array([0.0, 0.0, 0.0])  # [x, y, z]

        # due to a "bug" in the crazyflie's position estimator with the flow
        # deck that results in the estimator to reset the position to 0,0,0 mid
        # flight if there are changes in lighting or terrain (note: may also
        # be under other conditions, but so far only seen in those conditions)
        self._dynamic_home_xyz = np.array([0.0, 0.0, 0.0])  # [x, y, z]
        self._home_position_xyz = np.array([0.0, 0.0, 0.0])  # [x, y, z]
        self._cmd_position_xyz = np.array([0.0, 0.0, 0.0])  # the commanded position

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
        log_pos = LogConfig(name='LocalPosition', period_in_ms=150)
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

        log_vel = LogConfig(name='LocalVelocity', period_in_ms=100)
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

        # reset the estimator
        self._reset_position_estimator()

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

    def _send_command(self, cmd):
        """helper function to send the appropriate CF command

        based on the desired command type, send the corresponding setpoint command to the crazyflie.

        Args:
            cmd: the CrazyflieCommand to send
        """

        # based on the command type, send the appropriate crazyflie command
        if cmd.type == CrazyflieCommand.CMD_TYPE_VELOCITY:
            self._scf.cf.commander.send_velocity_world_setpoint(*cmd.cmd)

        elif cmd.type == CrazyflieCommand.CMD_TYPE_HOVER:
            # TODO: see if maybe want to get this current height information
            # back to the write loop...
            # current_height = cmd.cmd[3]
            self._scf.cf.commander.send_hover_setpoint(*cmd.cmd)

        elif cmd.type == CrazyflieCommand.CMD_TYPE_ATTITUDE_THRUST:
            self._scf.cf.commander.send_setpoint(*cmd.cmd)

        elif cmd.type == CrazyflieCommand.CMD_TYPE_ATTITUDE_DIST:
            # current_height = cmd.cmd[3]
            self._scf.cf.commander.send_zdistance_setpoint(*cmd.cmd)

        elif cmd.type == CrazyflieCommand.CMD_TYPE_STOP:
            # TODO: probably want to send appropriate flags that state disarmed, etc
            # TODO: basically need to update the drone state here
            self._scf.cf.commander.send_stop_setpoint()

        else:
            print("invalid command type!")

    def command_loop(self):
        """loop to send commands at a specified rate"""

        # the last time a command was sent
        # to be used to ensure commands are at the desired rate
        last_write_time = time.time()

        cmd_start_time = 0  # the time [s] that the command started -> needed for distance commands

        # the current command that should be being sent, default to 0 everything
        # current_cmd = CrazyflieCommand(CrazyflieCommand.CMD_TYPE_STOP, None)
        current_cmd = CrazyflieCommand(CrazyflieCommand.CMD_TYPE_ATTITUDE_THRUST, (0, 0, 0, 0), None)

        # the last commanded height
        # if this is not 0, want the hold commands to be hover to hold the specific height
        current_height = 0

        while self._running:

            # want to make sure the kalman filter has converged before sending any command
            while not self._converged:
                self._send_command(CrazyflieCommand(CrazyflieCommand.CMD_TYPE_ATTITUDE_THRUST, (0, 0, 0, 0), None))
                continue

            # empty out the queue of pending messages -> want to always send the messages asap
            # NOTE: the commands are immediately sent, which can result in fast back to back command sets, but
            # ensures all the commands are sent
            cmd = None
            while not self._out_msg_queue.empty():
                try:
                    cmd = self._out_msg_queue.get_nowait()
                except queue.Empty:
                    # if there is no msgs in the queue, will just continue
                    pass
                else:
                    if cmd is not None:
                        current_cmd = cmd
                        cmd_start_time = time.time()

                        # mark this entity as being parsed
                        self._out_msg_queue.task_done()

                        # convert the position command to a velocity (hover) command if needed
                        if cmd.type == CrazyflieCommand.CMD_TYPE_POSITION:
                            current_cmd = self._pos_cmd_to_cf_vel_cmd(np.array(cmd.cmd[0:3]), cmd.cmd[3])

                        # immediately handle the new command
                        self._send_command(current_cmd)

                        # DEBUG
                        # print("recevied command, type {}, cmd {}, delay {}".format(
                        # current_cmd.type, current_cmd.cmd, current_cmd.delay))

            # now that have handled any potentially new commands, let's handle the timer
            if current_cmd.delay is not None:
                if time.time() - cmd_start_time >= current_cmd.delay:
                    # DEBUG
                    # print("command timer completed, completed command: {}, {}".format(
                    # current_cmd.type, current_cmd.cmd))

                    # time to stop -> want to hold the commanded height (instead of the current height as current
                    # height may drift and that will cause the crazyflie to bob a lot)
                    current_height = self._cmd_position_xyz[2]
                    # print("stopping and holding a cmd height of {}".format(current_height))
                    if current_height > 0.05:
                        current_cmd = CrazyflieCommand(CrazyflieCommand.CMD_TYPE_HOVER, (0.0, 0.0, 0.0, current_height))
                    else:
                        current_cmd = CrazyflieCommand(CrazyflieCommand.CMD_TYPE_VELOCITY, (0.0, 0.0, 0.0, 0.0))

                    # now immediately handle the new command
                    self._send_command(current_cmd)

            # want to make sure that the commands are set at minimum specified rate
            # so send the command again if that rate timer requires it
            current_time = time.time()
            if (current_time - last_write_time) < (1.0 / self._send_rate):
                continue

            # resend the command and update the timestamp for when the last command was sent
            last_write_time = current_time
            self._send_command(current_cmd)

    def _cf_callback_pos(self, timestamp, data, logconf):
        """callback on the crazyflie's position update"""
        x = data['kalman.stateX']
        y = data['kalman.stateY']
        z = data['kalman.stateZ']
        # print("current height: {}".format(z))

        # compute a difference between the previou current position and this one
        # if there is a large jump, that means there is a chance the estimator has reset!
        new_position = np.array([x, y, z])

        dpos = new_position - self._current_position_xyz
        dx = dpos[0]
        dy = dpos[1]
        pos_change = math.sqrt(dx * dx + dy * dy)

        # DEBUG
        # print("position change: ({}, {})".format(dx, dy))

        # TODO: find the correct limit here for defining a jump
        if pos_change >= 1:
            # print("esitmator has reset, adjusting home position")
            self._dynamic_home_xyz += np.array([dx, dy, 0])
            # print("\tdynamic adjustment = ({}, {})".format(self._dynamic_home_xyz[0], self._dynamic_home_xyz[1]))

        self._current_position_xyz = np.array([x, y, z])  # save for our internal use

        # the position that should be published is an NED position, adjusted for the set home position
        adjusted_pos = self._current_position_xyz - self._home_position_xyz - self._dynamic_home_xyz
        pos = mt.LocalFrameMessage(timestamp, adjusted_pos[0], -adjusted_pos[1], -adjusted_pos[2])
        self.notify_message_listeners(MsgID.LOCAL_POSITION, pos)

    def _cf_callback_vel(self, timestamp, data, logconf):
        """callback on the crazyflie's velocity update"""

        if not self._converged:
            return

        x = data['kalman.statePX']
        y = data['kalman.statePY']
        z = data['kalman.statePZ']
        vel = mt.LocalFrameMessage(timestamp, x, -y, -z)
        self.notify_message_listeners(MsgID.LOCAL_VELOCITY, vel)

    def _cf_callback_att(self, timestamp, data, logconf):
        """callback on the crazyflie's attitude update"""

        if not self._converged:
            return

        roll = data['stabilizer.roll']
        pitch = data['stabilizer.pitch']
        yaw = data['stabilizer.yaw']
        fm = mt.FrameMessage(timestamp, roll, pitch, yaw)
        self.notify_message_listeners(MsgID.ATTITUDE, fm)

    def _cf_callback_state(self, timestamp, data, logconf):
        """callback on the crazyflie's state information"""
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
        """callback on the crazyflie's KF varaince information"""
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
        dx = max_x - min_x
        dy = max_y - min_y
        dz = max_z - min_z

        if dx < self._filter_threshold and dy < self._filter_threshold and dz < self._filter_threshold:
            print("filter has converge, position is good!")
            self._converged = True
            self._kf_log_config.stop()  # no longer care to keep getting the kalman filter variance

    def _cf_callback_error(self, logconf, msg):
        """callback for an error from one of the loggers"""
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
        # wait for the variance of the estimator to become small enough
        self._wait_for_position_estimator()

    def _convert_to_cf_xyz(self, pos):
        """convert the position to a position in the crazyflie's current frame

        handle the conversion from the user's drone frame to the frame being used in the crazyflie.

        Args:
            pos: numpy array of the desired position in the XYZ frame

        Returns:
            the XYZ position vector in the crazyflie's coordinate frame
            numpy array
        """
        return pos + self._dynamic_home_xyz + self._home_position_xyz

    def _create_velocity_cmd(self, dx, dy, z, heading):
        """helper function to create a velocity command given a desired change in position.

        computes the velocity command given the velocity setting and then passed in position change.
        also computes the necessary delay time to wait before finishing the command.

        Args:
            dx: distance to travel in the crazyflie's X direction
            dy: distance to travel in the crazyflie's Y direction
            dz: distance to travel in the crazyflie's Z direction
            z: the height above ground to hold
            heading: desired heading

        Returns:
            a velocity command (CF type HOVER) to execute the desired motion.
            CrazyflieCommand
        """

        # calculate the distance needed to travel and the delay time for the command
        distance = math.sqrt(dx * dx + dy * dy)
        delay_time = distance / self._velocity
        print("the delay time for the move command: {}".format(delay_time))

        # need to now calculate the velocity vector -> need to have a magnitude of default velocity
        vx = self._velocity * dx / distance
        vy = self._velocity * dy / distance
        # vz = self._velocity * dz / distance
        vz = 0
        print("vel vector: ({}, {}, {})".format(vx, vy, vz))

        # create and send the command
        # TODO: determine if would want to use the hover command instead of the velocity command....
        # TODO: problem with the hover command is have no feedback on the current altitude!!
        # return CrazyflieCommand(CrazyflieCommand.CMD_TYPE_VELOCITY, (vx, vy, vz, 0.0), delay_time)
        return CrazyflieCommand(CrazyflieCommand.CMD_TYPE_HOVER, (vx, vy, 0.0, z), delay_time)

    def _pos_cmd_to_cf_vel_cmd(self, cmd_pos_xyz, heading):
        """convert an XYZ command from the user to a velocity command in the crazyflie's frame

        handles the translation that needs to occur to take into account the set home position and any potential
        dynamic home adjustments needed due to the estimator onboard the crazyflie resetting.
        then computes the velocity command and delay time needed to reach the desired point, creating the necessary
        crazyflie command to successfully fly to the commanded position.

        Note: the user's XYZ frame is a frame with (0,0) at the location at which `set_home_position()` was called.

        Args:
            cmd_pos_xyz: the desired position in the user's XYZ frame
            heading: the desired vehicle heading

        Returns:
            a velocity move command (CF type hover) required to reach the desired position.
            CrazyflieCommand
        """

        # convert from the user's frame to the cf's internal frame
        cmd_pos_cf_xyz = self._convert_to_cf_xyz(cmd_pos_xyz)
        self._cmd_position_xyz = np.copy(cmd_pos_cf_xyz)

        # DEBUG - position info
        # print("current positions:")
        # print("\tvehicle: ({}, {}, {})".format(self._current_position_xyz[0], self._current_position_xyz[1],
        #                                        self._current_position_xyz[2]))
        # print("\thome: ({}, {}, {})".format(self._home_position_xyz[0], self._home_position_xyz[1],
        #                                     self._home_position_xyz[2]))
        # print("\tdynamic: ({}, {}, {})".format(self._dynamic_home_xyz[0], self._dynamic_home_xyz[1],
        #                                        self._dynamic_home_xyz[2]))

        # DEBUG - command info
        # print("command detailed:")
        # print("\tuser xyz frame: ({}, {}, {})".format(cmd_pos_xyz[0], cmd_pos_xyz[1], cmd_pos_xyz[2]))
        # print("\tcf frame: ({}, {}, {})".format(cmd_pos_cf_xyz[0], cmd_pos_cf_xyz[1], cmd_pos_cf_xyz[2]))

        # calculate the change vector needed
        # note the slight oddity that happens in converting NED to XYZ
        # as things are used as XYZ internally for the crazyflie
        dx = cmd_pos_cf_xyz[0] - self._current_position_xyz[0]
        dy = cmd_pos_cf_xyz[1] - self._current_position_xyz[1]
        # dz = cmd_pos_cf_xyz[2] - self._current_position_xyz[2]
        z = cmd_pos_cf_xyz[2]

        return self._create_velocity_cmd(dx, dy, z, heading)

    def set_velocity(self, velocity):
        """set the velocity the drone should use in flight"""
        self._velocity = velocity

    def arm(self):
        """Command to arm the drone"""
        # NOTE: this doesn't exist for the crazyflie
        pass

    def disarm(self):
        """Command to disarm the drone"""
        # NOTE: this doesn't exist for the crazyflie
        pass

    def take_control(self):
        """
        Command the drone to switch into a mode that allows external control.
        e.g. for PX4 this commands 'offboard' mode,
        while for APM this commands 'guided' mode
        """
        # NOTE: this doesn't exist for the crazyflie
        # however, if this command is being used, want to make sure
        # the state output conforms to the expected changes
        self._armed = True
        self._guided = True

    def release_control(self):
        """Command to return the drone to a manual mode"""
        # NOTE: this doesn't exist for the crazyflie
        # however, if this command is being used, want to make sure
        # the state output conforms to the expected changes
        self._armed = False
        self._guided = False

    def cmd_attitude(self, roll, pitch, yaw, thrust):
        """Command to set the desired attitude and thrust

        Args:
            yaw: the desired yaw in radians
            pitch: the desired pitch in radians
            roll: the deisred roll in radians
            thrust: the normalized desired thrust level on [0, 1]
        """

        # NOTE: for the crazyflie, the attitude commands are in degrees, so will need to adjust accordingly
        # TODO: crazyflie takes in roll/pitch/yaw, should figure out the impact of making this function correct
        # to the definition of cmd_attitude... not sure why commanding yaw rate here

        # NOTE: thrust is also a bit weird for the crazyflie, it's a value between 10001 and 60000
        # with hover thrust being around 36850.0

        roll_deg = np.degrees(roll)
        pitch_deg = -np.degrees(pitch)  # crazyflie is in an XYZ frame, so pitch direction is reversed
        yaw_deg = np.degrees(yaw)

        # map the thrust from [0, 1] to the crazyflie accepted [10000, 65000]
        thrust = thrust * 55000 + 10000

        # thrust needs to be an int
        thrust = int(thrust)

        # NOTE: again no delay time as that is not used when sending commands at this level
        self._out_msg_queue.put(CrazyflieCommand(CrazyflieCommand.CMD_TYPE_ATTITUDE_THRUST,
                                                 (roll_deg, pitch_deg, yaw_deg, thrust),
                                                 None))
        # self._out_msg_queue.put(CrazyflieCommand(CrazyflieCommand.CMD_TYPE_ATTITUDE_DIST,
        #                                          (roll_deg, pitch_deg, yaw_deg, 0.5),
        #                                          None))

    def cmd_attitude_zdist(self, roll, pitch, yaw, altitude):
        """Command to set the desired attitude and altitude.

        This is a custom crazyflie command that has the crazyflie worry about holding altitude and
        attitude is controlled by the user

        Args:
            roll: the desired roll in [radians]
            pitch: the desired pitch in [radians]
            yaw: the desired yaw in [radians]
            altitude: the desired altitude in [m]
        """

        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)

        # no time delay here
        # create the attitude zdistance command
        self._out_msg_queue.put(CrazyflieCommand(CrazyflieCommand.CMD_TYPE_ATTITUDE_DIST,
                                                 (roll_deg, pitch_deg, yaw_deg, altitude),
                                                 None))

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

        Note: For the crazyflie, NED is defined when the crazyflie starts, not aligned with world NED

        Args:
            vn: desired north velocity component in meters/second
            ve: desired east velocity component in meters/second
            vd: desired down velocity component in meters/second (note: positive down!)
            heading: desired drone heading in radians
        """

        # crazyflie works in an XYZ "world" frame, so need to convert from NED to XYZ
        vx = vn
        vy = -ve
        vz = -vd

        # TODO: crazyflie takes yaw_rate here, need to handle this correctly
        # for now, ignore all heading commands

        # for a velocity command the idea of a delay time doesn't exist, it's up to the user to make sure
        # that velocity commands keep getting sent
        delay_time = None

        self._out_msg_queue.put(CrazyflieCommand(CrazyflieCommand.CMD_TYPE_VELOCITY, (vx, vy, vz, 0.0), delay_time))

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

        # consider the waypoint as reached, so command the cf to stop
        current_height = self._cmd_position_xyz[2]
        stop_moving_cmd = CrazyflieCommand(CrazyflieCommand.CMD_TYPE_HOVER, (0.0, 0.0, 0.0, current_height))
        self._out_msg_queue.put(stop_moving_cmd)

        # need to know the current position: for now going to simply map NED to XYZ!!!
        # x is forward
        # y is left
        # z is up
        # also completely ignoring heading for now

        # send a position command - this will allow the write loop
        # to use the most up to date information for generating the
        # corresponding velocity command
        cmd = CrazyflieCommand(CrazyflieCommand.CMD_TYPE_POSITION, (n, -e, -d, heading))
        self._out_msg_queue.put(cmd)

        # # need to covert the commanded position to the crazyflie's
        # # "world" frame
        # cmd_pos_cf_xyz = self._convert_to_cf_xyz(cmd_pos_xyz)

        # # DEBUG - position info
        # print("current positions:")
        # print("\tvehicle: ({}, {}, {})".format(
        #     self._current_position_xyz[0],
        #     self._current_position_xyz[1],
        #     self._current_position_xyz[2]))
        # print("\thome: ({}, {}, {})".format(
        #     self._home_position_xyz[0],
        #     self._home_position_xyz[1],
        #     self._home_position_xyz[2]))
        # print("\tdynamic: ({}, {}, {})".format(
        #     self._dynamic_home_xyz[0],
        #     self._dynamic_home_xyz[1],
        #     self._dynamic_home_xyz[2]))

        # # DEBUG - command info
        # print("command detailed:")
        # print("\tuser xyz frame: ({}, {}, {})".format(n, -e, -d))
        # print("\tcf frame: ({}, {}, {})".format(
        #     cmd_pos_cf_xyz[0], cmd_pos_cf_xyz[1], cmd_pos_cf_xyz[2]))

        # # calculate the change vector needed
        # # note the slight oddity that happens in converting NED to XYZ
        # # as things are used as XYZ internally for the crazyflie
        # dx = cmd_pos_cf_xyz[0] - self._current_position_xyz[0]
        # dy = cmd_pos_cf_xyz[1] - self._current_position_xyz[1]
        # z = cmd_pos_cf_xyz[2]  # holding a specific altitude, so just pass altitude through directly

        # # DEBUG
        # # print("move vector: ({}, {}) at height {}".format(dx, dy, z))

        # # command the relative position
        # self.cmd_relative_position(dx, dy, z, heading)

    def cmd_relative_position(self, dx, dy, dz, heading):
        print("move vector: ({}, {}, {})".format(dx, dy, dz))

        # update the commanded position information
        # want to be able to keep track of the desired "world frame"
        # coordinates to be able to catch estimator errors.
        self._cmd_position_xyz = self._current_position_xyz + np.array([dx, dy, dz])

        # use the helper function for this
        cmd = self._create_velocity_cmd(dx, dy, self._cmd_position_xyz[2], heading)
        self._out_msg_queue.put(cmd)

        # distance = math.sqrt(dx * dx + dy * dy)
        # delay_time = distance / self._velocity
        # print("the delay time for the move command: {}".format(delay_time))

        # # need to now calculate the velocity vector -> need to have a magnitude of default velocity
        # vx = self._velocity * dx / distance
        # vy = self._velocity * dy / distance
        # print("vel vector: ({}, {})".format(vx, vy))

        # # create and send the command
        # # TODO: determine if would want to use the hover command instead of the velocity command....
        # # TODO: problem with the hover command is have no feedback on the current altitude!!
        # cmd = CrazyflieCommand(CrazyflieCommand.CMD_TYPE_HOVER, (vx, vy, 0.0, z), delay_time)
        # self._out_msg_queue.put(cmd)

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
        # self._reset_position_estimator()

        # set the command position
        self._cmd_position_xyz = np.copy(self._current_position_xyz)
        self._cmd_position_xyz[2] = -d

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

        # set the command position
        self._cmd_position_xyz = np.copy(self._current_position_xyz)
        self._cmd_position_xyz[2] = 0

        # need to know the current height here...
        current_height = self._current_position_xyz[2]
        decent_velocity = -self._velocity / 2  # [m/s]

        # calculate how long that command should be executed for
        # we aren't going to go all the way down before then sending a stop command
        # TODO: figure out a way to do this without sleeping!!
        delay_time = (current_height - 0.02) / (-1 * decent_velocity)  # the wait time in seconds

        # DEBUG
        # print("current height: {}, delay time: {}".format(current_height, delay_time))

        # make sure delay time is always positive and non-zero
        if delay_time < 0:
            delay_time = 0.1

        cmd = CrazyflieCommand(CrazyflieCommand.CMD_TYPE_VELOCITY, (0.0, 0.0, decent_velocity, 0.0), delay_time)
        self._out_msg_queue.put(cmd)

        # wait the desired amount of time and then send a stop command to kill the motors
        time.sleep(delay_time)
        self._out_msg_queue.put(CrazyflieCommand(CrazyflieCommand.CMD_TYPE_STOP, None))

    def set_home_position(self, lat, lon, alt):
        """Command to change the home position of the drone.

        Note: for the crazyflie, there is no global position coordinates.
        Therefore when this command is called, the current local position
        of the crazyflie will be used as the home position.
        **This will therefore ignore all input arguments!**

        Args:
            lat: desired home latitude in decimal degrees
            lon: desired home longitude in decimal degrees
            alt: desired home altitude in meters (AMSL)
        """

        # NOTE: for the crazyflie, this takes the current local position
        # and sets that value to home.
        # Therefore all of the inputs are ignored!

        # update the home position to be the current position
        # this will be added to all the waypoint commands to get the
        # proper coordinate to command
        self._home_position_xyz = self._current_position_xyz
        self._home_position_xyz[2] = 0.0  # for now keep this at 0

        # want to reset the dynamic home adjustment at this point, since resetting the home position
        self._dynamic_home_xyz = np.array([0.0, 0.0, 0.0])

        # DEBUG
        # print("home position set to be ({}, {}, {})\n".format(self._home_position_xyz[0], self._home_position_xyz[1],
        #                                                       self._home_position_xyz[2]))
