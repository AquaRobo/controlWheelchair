from zope.interface import implementer
from control.DTOs.motors import Motors
from control.interfaces.IMotorDriver import IMotorDriver
from utils.Dispatcher import Dispatcher

@implementer(IMotorDriver)
class MotorDriver:
    def __init__(self):
        self.commHandler = Dispatcher().get_communication_handler("ACTUATOR")

    def drive(self, motors_dict: dict[str, Motors]) -> None:
        motors_speeds = self.__buildMotorsArray(motors_dict)
        self.commHandler.sendData(motors_speeds)

    def __buildMotorsArray(self, motors_drict: dict[str, Motors]) -> list:
        """Converts motor dict to list and send its speed.
        Args:
            motors_dict: Dictionary holding motors objects
        Returns:
            str:  Motor's data formated in a list.
        Example:
            data = ["w", motor.current_speed, .....for number of motors]
        """
        data = ["w"]
        for motor in motors_drict.values():
            data.extend([float(motor.current_pwm)]) 
        return data
