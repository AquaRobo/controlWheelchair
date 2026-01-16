from zope.interface import implementer
from control.DTOs.motors import Motors
from control.interfaces.IMotorDriver import IMotorDriver
from utils.Dispatcher import Dispatcher
import json

@implementer(IMotorDriver)
class MotorDriver:
    def __init__(self):
        self.commHandler = Dispatcher().get_communication_handler("ACTUATOR")

    def drive(self, motors_dict: dict[str, Motors]) -> None:
        motors_json = self.__buildMotorsArray(motors_dict)
        self.commHandler.sendData(motors_json)

    def __buildMotorsArray(self, motors_dict: dict[str, Motors]) -> json:
        """Converts motor dict to json array to identify each motor by its index
        and send its pwm and dir values.
        Args:
            motors_dict (dict[str, Motors]): Dictionary of motor instances.
        Returns:
            json: JSON array of motors with their pwm and dir values.
        Example:
            data = {"RIGHT_MOTORS": {pwm: 0, dir: 1}, "LEFT_MOTORS": {pwm: 0, dir: 1}}
        """
        control_motors_dict = {}
        for motor_side, motor in motors_dict.items():
            control_motors_dict[motor_side] = {"pwm": motor.current_pwm, "dir": motor.dir_bit}
        return json.dumps(control_motors_dict)
