import struct
from zope.interface import implementer
from control.DTOs.motors import Motors
from control.interfaces.IMotorDriver import IMotorDriver
from utils.Dispatcher import Dispatcher

_START_FRAME = 0xABCD
_MAX_SPEED   = 1000
_MIN_SPEED   = -1000

@implementer(IMotorDriver)
class MotorDriver:
    def __init__(self):
        self.commHandler = Dispatcher().get_communication_handler("STM")

    def drive(self, motors_dict: dict[str, Motors]) -> None:
        motors_speeds = self.__buildMotorsArray(motors_dict)
        
        self.commHandler.sendData(motors_speeds)

    def __buildMotorsArray(self, motors_dict: dict[str, Motors]) -> bytes:
        """Builds an 8-byte hoverboard command frame from motor PWM values.

        Frame layout (little-endian):
            uint16  START    = 0xABCD
            int16   steer    (left motor PWM, clamped to -1000..+1000)
            int16   speed    (right motor PWM, clamped to -1000..+1000)
            uint16  checksum = START ^ steer ^ speed

        Args:
            motors_dict: Dictionary holding Motors objects keyed by motor name.
        Returns:
            bytes: 8-byte frame ready to send over UART.
        """
        steer = 0
        speed = 0
        for motor in motors_dict.values():
            value = int(max(_MIN_SPEED, min(_MAX_SPEED, motor.current_pwm)))
            if motor.side == "left":
                steer = value
            elif motor.side == "right":
                speed = value

        checksum = _START_FRAME ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)
        return struct.pack('<HhhH', _START_FRAME, steer, speed, checksum)
