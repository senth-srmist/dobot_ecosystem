from __future__ import annotations

import struct
from typing import TYPE_CHECKING, Literal

if TYPE_CHECKING:
    from ..dobot import Dobot

from ..dobot_message import Message
from ..exceptions import DobotException


class ConveyorBelt:
    def __init__(self, bot: Dobot):
        self.bot = bot
        self.current_speed: float = 0.0

    def move(self, speed: float, interface: Literal[0, 1] = 0) -> None:
        """
        Run conveyor belt.

        speed:
            -1.0 = maximum reverse
             0.0 = stop
             1.0 = maximum forward

        interface:
            0 = Stepper1
            1 = Stepper2
        """

        if not -1.0 <= speed <= 1.0:
            raise DobotException(
                "Speed must be between -1.0 and 1.0"
            )

        # Convert normalized speed to Dobot stepper speed.
        # 1000 is the value we have verified experimentally.
        motor_speed = int(speed * 10000)

        self.current_speed = speed

        self._set_stepper_motor(
            speed=motor_speed,
            interface=interface,
            motor_control=True
        )

    def idle(self):
        """Stop the conveyor belt."""

        self._set_stepper_motor(
            speed=0,
            interface=0,
            motor_control=True
        )

        self.current_speed = 0.0

    def _set_stepper_motor(
        self,
        speed: int,
        interface: int = 0,
        motor_control: bool = True
    ) -> Message:

        msg = Message()

        # SetStepperMotor command
        msg.id = 0x87
        msg.ctrl = 0x03

        msg.params = bytearray()

        # Stepper interface
        # 0 = Stepper1
        # 1 = Stepper2
        msg.params.extend(
            bytearray([0x01 if interface == 1 else 0x00])
        )

        # Motor enable
        msg.params.extend(
            bytearray([0x01 if motor_control else 0x00])
        )

        # Signed 32-bit little-endian speed
        msg.params.extend(
            struct.pack("<i", int(speed))
        )

        return self.bot._send_command(msg)

