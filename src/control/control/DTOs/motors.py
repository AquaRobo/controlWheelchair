#!/usr/bin/env python3

class Motors:
    def __init__(self, motors_config: dict):
        self.side = motors_config["SIDE"]
        self.location = motors_config["LOCATION"]
        self.min_speed = motors_config["MIN_SPEED"]
        self.max_speed = motors_config["MAX_SPEED"]
        self.min_rpm = motors_config["MIN_RPM"]
        self.max_rpm = motors_config["MAX_RPM"]
        self.current_speed = self.min_speed
        self.target_speed = self.min_speed
        self.current_pwm = self.min_speed
        self.target_pwm = self.min_speed
        self.dir_bit = 0 if self.side == "right" else 1
        self.pwm_pin = motors_config["PWM_PIN"]
        self.dir_pin = motors_config["DIR_PIN"]
