from zope.interface import Interface

class ISteering(Interface):

    def steer(motors_dict: dict[str, object]) -> None:
        """Calculate and update motor commands based on robot speed.
        
        Args:
            motors_dict (dict[str, object]): A dictionary containing motor instances.
        Returns:
            None: Updates the motors_dict in place.
            
        This method should determine whether to set dir_bit to 0 or 1
        for each motor based on direction logic like forward/reverse or
        turning left/right.
        """