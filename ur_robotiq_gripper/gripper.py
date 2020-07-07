"""
MIT License

Copyright (c) 2019 Anders Prier Lindvig - SDU Robotics
Copyright (c) 2020 Fabian Freihube - DavinciKitchen GmbH

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Module to control Robotiq's gripper 2F-85 and Hand-E.
Originally from here: https://sdurobotics.gitlab.io/ur_rtde/_static/gripper_2f85.py
Adjusted for use with asyncio
"""

import asyncio
from enum import Enum
from typing import Union, Tuple, OrderedDict


class Gripper:
    """
    Communicates with the gripper directly, via socket with string commands, leveraging string names for variables.
    """

    # WRITE VARIABLES (CAN ALSO READ)
    ACT = "ACT"  # act : activate (1 while activated, can be reset to clear fault status)
    GTO = "GTO"  # gto : go to (will perform go to with the actions set in pos, for, spe)
    ATR = "ATR"  # atr : auto-release (emergency slow move)
    ADR = "ADR"  # adr : auto-release direction (open(1) or close(0) during auto-release)
    FOR = "FOR"  # for : force (0-255)
    SPE = "SPE"  # spe : speed (0-255)
    POS = "POS"  # pos : position (0-255), 0 = open
    # READ VARIABLES
    STA = "STA"  # status (0 = is reset, 1 = activating, 3 = active)
    PRE = "PRE"  # position request (echo of last commanded position)
    OBJ = "OBJ"  # object detection (0 = moving, 1 = outer grip, 2 = inner grip, 3 = no object at rest)
    FLT = "FLT"  # fault (0=ok, see manual for errors if not zero)

    ENCODING = "UTF-8"  # ASCII and UTF-8 both seem to work

    class GripperStatus(Enum):
        """Gripper status reported by the gripper. The integer values have to match what the gripper sends."""

        RESET = 0
        ACTIVATING = 1
        # UNUSED = 2  # This value is currently not used by the gripper firmware
        ACTIVE = 3

    class ObjectStatus(Enum):
        """Object status reported by the gripper. The integer values have to match what the gripper sends."""

        MOVING = 0
        STOPPED_OUTER_OBJECT = 1
        STOPPED_INNER_OBJECT = 2
        AT_DEST = 3

    def __init__(self, hostname: str, port: int = 63352) -> None:
        """Constructor.

        :param hostname: Hostname or ip of the robot arm.
        :param port: Port.

        """
        self.socket_reader = None
        self.socket_writer = None
        self.command_lock = asyncio.Lock()
        self._min_position = 0
        self._max_position = 255
        self._min_speed = 0
        self._max_speed = 255
        self._min_force = 0
        self._max_force = 255

        self.hostname = hostname
        self.port = port

    async def connect(self) -> None:
        """Connects to a gripper on the provided address"""
        self.socket_reader, self.socket_writer = await asyncio.open_connection(self.hostname, self.port)

    async def disconnect(self) -> None:
        """Closes the connection with the gripper."""
        self.socket_writer.close()
        await self.socket_writer.wait_closed()

    async def _set_vars(self, var_dict: OrderedDict[str, Union[int, float]]) -> bool:
        """Sends the appropriate command via socket to set the value of n variables, and waits for its 'ack' response.

        :param var_dict: Dictionary of variables to set (variable_name, value).
        :return: True on successful reception of ack, false if no ack was received, indicating the set may not
        have been effective.
        """
        # construct unique command
        cmd = "SET"
        for variable, value in var_dict.items():
            cmd += f" {variable} {str(value)}"
        cmd += "\n"  # new line is required for the command to finish
        # atomic commands send/rcv
        async with self.command_lock:
            self.socket_writer.write(cmd.encode(self.ENCODING))
            await self.socket_writer.drain()
            response = await self.socket_reader.read(1024)
        return self._is_ack(response)

    async def _set_var(self, variable: str, value: Union[int, float]) -> bool:
        """Sends the appropriate command via socket to set the value of a variable, and waits for its 'ack' response.

        :param variable: Variable to set.
        :param value: Value to set for the variable.
        :return: True on successful reception of ack, false if no ack was received, indicating the set may not
        have been effective.
        """
        return await self._set_vars(OrderedDict([(variable, value)]))

    async def _get_var(self, variable: str) -> int:
        """Sends the appropriate command to retrieve the value of a variable from the gripper, blocking until the
        response is received or the socket times out.

        :param variable: Name of the variable to retrieve.
        :return: Value of the variable as integer.
        """
        # atomic commands send/rcv
        async with self.command_lock:
            cmd = f"GET {variable}\n"
            self.socket_writer.write(cmd.encode(self.ENCODING))
            await self.socket_writer.drain()
            data = await self.socket_reader.read(1024)

        # expect data of the form 'VAR x', where VAR is an echo of the variable name, and X the value
        # note some special variables (like FLT) may send 2 bytes, instead of an integer. We assume integer here
        var_name, value_str = data.decode(self.ENCODING).split()
        if var_name != variable:
            raise ValueError(f"Unexpected response {data} ({data.decode(self.ENCODING)}): does not match '{variable}'")
        value = int(value_str)
        return value

    @staticmethod
    def _is_ack(data: str) -> bool:
        return data == b"ack"

    async def activate(self, auto_calibrate: bool = True) -> None:
        """Resets the activation flag in the gripper, and sets it back to one, clearing previous fault flags.

        :param auto_calibrate: Whether to calibrate the minimum and maximum positions based on actual motion.
        """
        # clear and then reset ACT
        await self._set_var(self.STA, 0)
        await self._set_var(self.STA, 1)

        # wait for activation to go through
        while not await self.is_active():
            await asyncio.sleep(0.01)

        # auto-calibrate position range if desired
        if auto_calibrate:
            await self.auto_calibrate()

    async def is_active(self) -> bool:
        """Returns whether the gripper is active."""
        status = await self._get_var(self.STA)
        return Gripper.GripperStatus(status) == Gripper.GripperStatus.ACTIVE

    def get_min_position(self) -> int:
        """Returns the minimum position the gripper can reach (open position)."""
        return self._min_position

    def get_max_position(self) -> int:
        """Returns the maximum position the gripper can reach (closed position)."""
        return self._max_position

    def get_open_position(self) -> int:
        """Returns what is considered the open position for gripper (minimum position value)."""
        return self.get_min_position()

    def get_closed_position(self) -> int:
        """Returns what is considered the closed position for gripper (maximum position value)."""
        return self.get_max_position()

    async def is_open(self) -> bool:
        """Returns whether the current position is considered as being fully open."""
        return await self.get_current_position() <= self.get_open_position()

    async def is_closed(self) -> bool:
        """Returns whether the current position is considered as being fully closed."""
        return await self.get_current_position() >= self.get_closed_position()

    async def get_current_position(self) -> int:
        """Returns the current position as returned by the physical hardware."""
        return await self._get_var(self.POS)

    async def auto_calibrate(self, log: bool = True) -> None:
        """Attempts to calibrate the open and closed positions, by slowly closing and opening the gripper.

        :param log: Whether to print the results to log.
        """
        # first try to open in case we are holding an object
        (position, status) = await self.move_and_wait_for_pos(self.get_open_position(), 64, 1)
        if Gripper.ObjectStatus(status) != Gripper.ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed opening to start: {str(status)}")

        # try to close as far as possible, and record the number
        (position, status) = await self.move_and_wait_for_pos(self.get_closed_position(), 64, 1)
        if Gripper.ObjectStatus(status) != Gripper.ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed because of an object: {str(status)}")
        assert position <= self._max_position
        self._max_position = position

        # try to open as far as possible, and record the number
        (position, status) = await self.move_and_wait_for_pos(self.get_open_position(), 64, 1)
        if Gripper.ObjectStatus(status) != Gripper.ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed because of an object: {str(status)}")
        assert position >= self._min_position
        self._min_position = position

        if log:
            # TODO: remove prints, replace by pyton logger
            print(f"Gripper auto-calibrated to [{self.get_min_position()}, {self.get_max_position()}]")

    async def move(self, position: int, speed: int, force: int) -> Tuple[bool, int]:
        """Sends commands to start moving towards the given position, with the specified speed and force.

        :param position: Position to move to [min_position, max_position]
        :param speed: Speed to move at [min_speed, max_speed]
        :param force: Force to use [min_force, max_force]
        :return: A tuple with a bool indicating whether the action it was successfully sent, and an integer with
        the actual position that was requested, after being adjusted to the min/max calibrated range.
        """

        def clip_val(min_val, val, max_val):
            return max(min_val, min(val, max_val))

        clip_pos = clip_val(self._min_position, position, self._max_position)
        clip_spe = clip_val(self._min_speed, speed, self._max_speed)
        clip_for = clip_val(self._min_force, force, self._max_force)

        # moves to the given position with the given speed and force
        var_dict = OrderedDict([(self.POS, clip_pos), (self.SPE, clip_spe), (self.FOR, clip_for), (self.GTO, 1)])
        return await self._set_vars(var_dict), clip_pos

    async def move_and_wait_for_pos(self, position: int, speed: int, force: int) -> Tuple[int, ObjectStatus]:  # noqa
        """Sends commands to start moving towards the given position, with the specified speed and force, and
        then waits for the move to complete.

        :param position: Position to move to [min_position, max_position]
        :param speed: Speed to move at [min_speed, max_speed]
        :param force: Force to use [min_force, max_force]
        :return: A tuple with an integer representing the last position returned by the gripper after it notified
        that the move had completed, a status indicating how the move ended (see ObjectStatus enum for details). Note
        that it is possible that the position was not reached, if an object was detected during motion.
        """
        set_ok, cmd_pos = await self.move(position, speed, force)
        if not set_ok:
            raise RuntimeError("Failed to set variables for move.")

        # wait until the gripper acknowledges that it will try to go to the requested position
        while await self._get_var(self.PRE) != cmd_pos:
            await asyncio.sleep(0.001)

        # wait until not moving
        cur_obj = await self._get_var(self.OBJ)
        # TODO: add timeout
        while Gripper.ObjectStatus(cur_obj) == Gripper.ObjectStatus.MOVING:
            cur_obj = await self._get_var(self.OBJ)

        # report the actual position and the object status
        final_pos = await self._get_var(self.POS)
        final_obj = cur_obj
        return final_pos, Gripper.ObjectStatus(final_obj)
