#!/usr/bin/env python3
from control.DTOs.motors import Motors

class PWMMapper:
    """Maps wheel target speeds to hoverboard PWM values.

    Linearly scales each motor's target_speed (bounded by the motor's
    min_rpm/max_rpm from motors.yaml) into the hoverboard PWM range
    (±min_speed..±max_speed, e.g. ±300..±800), preserving sign and mapping
    zero speed to zero PWM. Raises ValueError if a speed is out of range.

    Input:  motors_dict with target_speed set (by MotorSpeedEvaluator).
    Output: sets motor.target_pwm on each motor in place.
    """

    @staticmethod
    def mapAxesToPWM(motors:dict[str, Motors]) -> None:
        """Map target speeds [-8.5, 8.5] to hooverboard's motors PWM values:
            positive:  300 to 800
            negative: -300 to -800
            zero: 0
        Args:
            motors (dict[str, Motors]): Dictionary of motor instances."""
        for motor in motors.values():
            if not (motor.min_rpm <= motor.target_speed <= motor.max_rpm):
                raise ValueError(f"Speed value {motor.target_speed} out of range [{motor.min_rpm}, {motor.max_rpm}] for motor on side {motor.side}")
            if motor.target_speed > 0:
                motor.target_pwm = int(motor.min_speed + (motor.max_speed - motor.min_speed) * (motor.target_speed / motor.max_rpm))
            elif motor.target_speed < 0:
                motor.target_pwm = int(-motor.min_speed + (motor.max_speed - motor.min_speed) * (motor.target_speed / motor.max_rpm))
            elif motor.target_speed == 0:
                motor.target_pwm = 0

            