from simple_pid import PID

class PIDController:
    """A simple PID controller wrapper using the simple-pid library."""
    def __init__(self, kp: float, ki: float, kd: float, setpoint: float = None):
        self.setpoint = setpoint
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._pid = PID(kp, ki, kd, setpoint=setpoint)
        self._pid.output_limits = (-1, 1)  # Set output limits to match angular velocity of the robot
        self._pid.sample_time = 0.01  # Sample time in seconds

    def updateSetpoint(self, setpoint: float):
        """
        Update the setpoint for the PID controller.

        Parameters:
            setpoint (float): The new setpoint.
        """
        self.setpoint = setpoint
        self._pid.setpoint = setpoint

    def updateConstants(self, kp: float, ki: float, kd: float):
        """
        Update the gains for the PID controller.

        Parameters:
            kp (float): The new Proportional gain.
            ki (float): The new Integral gain.
            kd (float): The new Differential gain.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._pid.Kp = kp
        self._pid.Ki = ki
        self._pid.Kd = kd

    def stablize(self, measured_value: float) -> float:
        """
        Calculate the control output based on the measured value.

        Parameters:
            measured_value (float): The current measured value.

        Returns:
            float: The control output.
        """
        if measured_value is not None and self._pid.setpoint is not None:
            error = self._angleDifference(measured_value, self._pid.setpoint)
            control = self._pid(measured_value + error)
            return control

    def _angleDifference(self, a: float, b: float) -> float:
        """
        Calculate the shortest difference between two angles as the IMU range is in cyclic angles [-180,180].
    
        Parameters:
            a (float): The first angle [Actual measurement].
            b (float): The second angle [Target measurement].
    
        Returns:
            float: The smallest difference between the two angles.
        """
        diff = (a - b + 180) % 360 - 180  
        return diff