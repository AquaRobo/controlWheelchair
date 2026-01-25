from zope.interface import implementer
from control.DTOs.motors import Motors
from control.interfaces.IMotorDriver import IMotorDriver
from utils.Dispatcher import Dispatcher

@implementer(IMotorDriver)
class MotorDriver:
    def __init__(self):
        self.commHandler = Dispatcher().get_communication_handler("ACTUATOR")

    def drive(self, motors_dict: dict[str, Motors]) -> None:
        for motor_name, motor in motors_dict.items():
            motors_msg = self.__buildMotorsMessage(motor)
        self.commHandler.sendData(motors_msg)

    def __buildMotorsMessage(self, motor: Motors) -> str:
        """Converts motor dict to string and send its pwm and dir values.
        Args:
            motor: Motor instance holding all its data.
        Returns:
            str:  Motor's data formated as a string.
        Example:
            message = "{motor.pwm_pin}-{motor.dir_pin}-{motor.current_pwm}-{motor.dir_bit}"
        """
        message = f"{motor.pwm_pin}-{motor.dir_pin}-{motor.current_pwm}-{motor.dir_bit}"
        return message
