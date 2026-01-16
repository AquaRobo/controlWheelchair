from zope.interface import implementer
from control.interfaces.ISteering import ISteering
from control.DTOs.motors import Motors

@implementer(ISteering)
class DifferentialDriveSteering:
    
    @staticmethod
    def steer(motors_dict: dict[str, Motors]) -> None:
        """Calculate and update motor commands based on robot speed.
        
        Args:
            motors_dict (dict[str, Motors]): A dictionary containing motor instances.
        """
        # Get lists of left and right motors
        left_motors = [m for m in motors_dict.values() if m.side == "left"]
        right_motors = [m for m in motors_dict.values() if m.side == "right"]

        if not left_motors or not right_motors:
            return

        for motor in right_motors:
            motor.dir_bit = 0 if motor.speed > 0 else 1
        
        for motor in left_motors:
            motor.dir_bit = 1 if motor.speed > 0 else 0
        