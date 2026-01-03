from zope.interface import Interface
from control.DTOs.motors import Motors

class ISpeedEvaluator(Interface):
    def evaluateSpeeds(x_axis: float, z_axis: float, yaw_axis: float, motors_dict: dict[str, Motors]) -> None:
        """Evaluate and set the speeds for each motor based on joystick inputs.
        
        Args:
            x_axis (float): The x-axis value from the joystick (-1.0 to 1.0).
            z_axis (float): The z-axis value from the joystick (-1.0 to 1.0).
            yaw_axis (float): The yaw-axis value from the joystick (-1.0 to 1.0).
            motors_dict (dict[str, Motors]): A dictionary containing motor instances.
        """
        pass