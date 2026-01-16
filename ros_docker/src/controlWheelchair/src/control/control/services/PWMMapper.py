#!/usr/bin/env python3
from control.DTOs.motors import Motors

class PWMMapper:

    @staticmethod
    def mapAxesToPWM(motors:dict[str, Motors]) -> None:
        """Convert all motor speeds from -10.6-10.6 range to 0-255 range for PWM control.
        Args:
            motors (dict[str, Motors]): Dictionary of motor instances."""
        for motor in motors.values():
            # if not (-10.6 <= motor.speed <= 10.6):
            #     raise ValueError(f"Speed value {motor.speed} out of range [-10.6, 10.6] for motor on side {motor.side}")

            motor.target_pwm = int(motor.min_speed + (motor.max_speed - motor.min_speed) * (abs(motor.speed) / 10.6))