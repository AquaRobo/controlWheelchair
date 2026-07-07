class OdometryEvaluator:
    """Stateless differential-drive odometry math (rear-wheel model).

    Pure static helpers shared by OdomNode and OdomHardwareNode:
    wheel position deltas -> wheel rotational speeds -> robot linear/angular
    velocity and pose increments. Only the rear (driven) wheels are used, so
    the model matches MotorSpeedEvaluator's command kinematics.

    Input:  wheel angular displacements [rad], dt [ns], wheel radius and
            separation [m].
    Output: (fi_left, fi_right) [rad/s], (linear, angular) robot velocities
            [m/s, rad/s], (d_s, d_theta) pose increments [m, rad].
    """

    @staticmethod
    def getRotationalSpeeds(dp_left: float, dp_right: float, dt: float) -> tuple[float, float]:
        """Calculate the rotational speeds of the left and right wheels.
        
        Args:
            dp_left (float): Change in position of the left wheel.
            dp_right (float): Change in position of the right wheel.
            dt (float): Time interval in nanoseconds.
        Returns:
            fi_left (float): speed of the left wheel
            fi_right (float): speed of the right wheel
        """
        try:
            fi_left = dp_left / (dt / 1e9)
            fi_right = dp_right / (dt / 1e9)
            return fi_left, fi_right
        except ZeroDivisionError:
            return 0.0, 0.0
    
    @staticmethod
    def getRobotVelocities(fi_rear_left: float, fi_rear_right: float, wheel_radius: float, wheel_separation: float) -> tuple[float, float]:
        """Calculate robot linear/angular velocity matching MotorSpeedEvaluator's rear-drive model.

        Rear wheels are the driven pair; front wheels are passive casters. Velocities are
        derived only from the rear wheel speeds to stay consistent with the command model.
        """
        linear = wheel_radius * (fi_rear_right + fi_rear_left) / 2.0
        angular = wheel_radius * (fi_rear_right - fi_rear_left) / wheel_separation
        return linear, angular
    
    @staticmethod
    def getPositionIncrement(dp_rear_left: float, dp_rear_right: float, rear_wheel_radius: float, rear_wheel_separation: float) -> tuple[float, float]:
        """Calculate the position increment of the robot.
        
        Args:
            dp_left (float): Change in position of the left wheel.
            dp_right (float): Change in position of the right wheel.
            wheel_radius (float): Radius of the wheels.
            wheel_separation (float): Distance between the two wheels.
        Returns:
            d_s (float): Linear position increment of the robot.
            d_theta (float): Angular position increment of the robot.
        """
        d_s = rear_wheel_radius * (dp_rear_right + dp_rear_left) / 2.0
        d_theta = rear_wheel_radius * (dp_rear_right - dp_rear_left) / rear_wheel_separation
        return d_s, d_theta
    
