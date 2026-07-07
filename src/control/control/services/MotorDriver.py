from zope.interface import implementer
from control.DTOs.motors import Motors
from control.interfaces.IMotorDriver import IMotorDriver
from utils.Dispatcher import Dispatcher
import struct

START_FRAME = 0xABCD    # uint16 start marker

@implementer(IMotorDriver)
class MotorDriver:
    """Sends motor PWM commands to the hoverboard controller (STM) frame-encoded.

    Implements IMotorDriver. Packs commands into the hoverboard UART protocol:
    little-endian <start(0xABCD), right(int16), left(int16), checksum(XOR)> and
    writes them through the communication handler resolved for "STM" (UART).

    Input:  drivePwm(right_pwm, left_pwm) — PWM values, or drive(motors_dict)
            using each motor's current_pwm.
    Output: 8-byte command frame on the STM communication channel; feedback is
            read back separately by HoverBoardNode via commHandler.receiveData().
    """

    def __init__(self):
        self.commHandler = Dispatcher().get_communication_handler("STM")

    def drive(self, motors_dict: dict[str, Motors]) -> None:
        motors_speeds = self.__buildMotorsArray(motors_dict)
        self.commHandler.sendData(motors_speeds)

    def drivePwm(self, right_pwm: float, left_pwm: float) -> None:
        right = int(right_pwm)
        left = int(left_pwm)
        checksum = START_FRAME ^ (right & 0xFFFF) ^ (left & 0xFFFF)
        data = struct.pack('<HhhH', START_FRAME, right, left, checksum)
        self.commHandler.sendData(data)

    def __buildMotorsArray(self, motors_dict: dict[str, Motors]) -> list:
        """Converts motor dict to list and send its speed.
        Args:
            motors_dict: Dictionary holding motors objects
        Returns:
            list: Motor's data formatted in a list.
        Example:
            data = ["w", motor.current_speed, .....for number of motors]
        """
        data = []
        for motor in motors_dict.values():
            data.extend([int(motor.current_pwm)])
        
        checksum = START_FRAME ^ (data[-1] & 0xFFFF) ^ (data[-2] & 0xFFFF)
        return struct.pack('<HhhH', START_FRAME, data[-1], data[-2], checksum)


