from zope.interface import implementer
from control.DTOs.motors import Motors
from control.interfaces.IMotorDriver import IMotorDriver
from utils.Dispatcher import Dispatcher
import struct

START_FRAME = 0xABCD    # uint16 start marker

@implementer(IMotorDriver)
class MotorDriver:
    def __init__(self):
        self.commHandler = Dispatcher().get_communication_handler("STM")

    def drive(self, motors_dict: dict[str, Motors]) -> None:
        motors_speeds = self.__buildMotorsArray(motors_dict)
        self.commHandler.sendData(motors_speeds)

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


