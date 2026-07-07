from control.interfaces.ISpeedEvaluator import ISpeedEvaluator
from utils.Configurator import Configurator
from zope.interface import implementer
from control.DTOs.motors import Motors
import numpy as np

@implementer(ISpeedEvaluator)
class MotorSpeedEvaluator:
    """Inverse kinematics: robot velocity to per-wheel angular speeds.

    Implements ISpeedEvaluator for the differential-drive wheelchair. Solves the
    rear (driven) wheel speeds from the commanded linear/angular velocity, and
    computes matching speeds for the passive front casters at their track
    positions. A PID yaw correction is folded into the angular velocity before
    solving. Wheel geometry is read from wheelchair_config.yaml.

    Input:  x_axis [m/s], z_axis [rad/s], yaw_axis (PID output), motors_dict.
    Output: sets motor.target_speed [rad/s] on every motor in motors_dict.
    """

    def __init__(self):
        self.robot_config = Configurator("control").fetchData(Configurator.WHEELCHAIR_CONFIG)
        self.rear_wheels_radius = self.robot_config['rear_wheels_radius']
        self.rear_wheels_separation = self.robot_config['rear_wheels_separation']
        self.front_wheels_radius = self.robot_config['front_wheels_radius']
        self.front_wheels_separation = self.robot_config['front_wheels_separation']
        self.rear_speed_conversion = np.array([
            [self.rear_wheels_radius/2.0, self.rear_wheels_radius/2.0],
            [-self.rear_wheels_radius/self.rear_wheels_separation, self.rear_wheels_radius/self.rear_wheels_separation]
        ], dtype=float)
        self._half_front_track = self.front_wheels_separation / 2.0
        
    def evaluateSpeeds(self, x_axis: float, z_axis: float, yaw_axis: float, motors_dict: dict[str, Motors]) -> None:
        """Evaluate and set the speed for each motor based on robot speed using forward kinematics.
        
        Args:
            x_axis (float): Linear velocity of the robot.
            z_axis (float): Angular velocity of the robot.
            yaw_axis (float): PID Control signal.
            motors_dict (dict[str, Motors]): A dictionary containing motor instances.
        Returns:
            None: Updates the motors_dict in place.
        """
        if yaw_axis < 0:
            angular_velocity = z_axis - yaw_axis
        else:
            angular_velocity = z_axis + yaw_axis
        robot_speed = np.array([x_axis, angular_velocity], dtype=float)
        rear_left_speed, rear_right_speed = np.linalg.solve(self.rear_speed_conversion, robot_speed)

        # Front wheels are passive: match linear velocity at their track positions
        v_left_front = x_axis - angular_velocity * self._half_front_track
        v_right_front = x_axis + angular_velocity * self._half_front_track
 
        front_left_speed = v_left_front / self.front_wheels_radius
        front_right_speed = v_right_front / self.front_wheels_radius

        for motor in motors_dict.values():
            if motor.location == "rear":
                motor.target_speed = rear_left_speed if motor.side == "left" else rear_right_speed
            elif motor.location == "front":
                motor.target_speed = front_left_speed if motor.side == "left" else front_right_speed