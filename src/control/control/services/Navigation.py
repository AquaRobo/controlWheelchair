from control.interfaces.ISpeedEvaluator import ISpeedEvaluator
from control.services.PWMMapper import PWMMapper
from control.DTOs.motors import Motors
from utils.Dispatcher import Dispatcher
from utils.Configurator import Configurator

class Navigation:
    """Drive pipeline service: velocity command in, wheel speeds/PWM out.

    Orchestrates one navigation step for NavigationNode: the speed evaluator
    computes per-wheel target speeds from (x, z, yaw) inputs, PWMMapper converts
    them to hoverboard PWM values, and the configured smoothing strategy ramps
    current speed/PWM toward the targets. Motor definitions are loaded from
    motors.yaml; steering/smoothing strategies come from the Dispatcher.

    Input:  x_axis (linear vel), z_axis (angular vel), yaw_axis (PID correction)
            via navigate().
    Output: getMotorsSpeed() — smoothed wheel speeds [rad/s] for simulation;
            getMotorsPWM() — smoothed (right, left) PWM for the hardware driver.
    """

    def __init__(self, speed_evaluator: ISpeedEvaluator):
        motors_yaml_data = Configurator("control").fetchData(Configurator.MOTORS)
        self.speed_evaluator = speed_evaluator
        self.steering_strat = Dispatcher().get_steering_strategy()
        self.smoothing_strat = Dispatcher().get_smoothing_strategy()
        # Convert YAML dict to Motor objects
        self.motors_dict = self.__toMotorObjects(motors_yaml_data)

    def navigate(self, x_axis: float, z_axis: float, yaw_axis: float) -> None:
        self.speed_evaluator.evaluateSpeeds(x_axis, z_axis, yaw_axis, self.motors_dict)
        # self.steering_strat.steer(self.motors_dict)
        PWMMapper().mapAxesToPWM(self.motors_dict)
        for motor_name, motor in self.motors_dict.items():
            motor.current_speed = self.smoothing_strat.smooth(motor.current_speed, motor.target_speed)
            target_pwm = motor.target_pwm
            smoothed_pwm = self.smoothing_strat.smooth(motor.current_pwm, target_pwm)
            motor.current_pwm = smoothed_pwm

    def getMotorsSpeed(self):
        speeds = []
        for motor_name, motor in self.motors_dict.items():
            speeds.append(motor.current_speed)
        return speeds

    def getMotorsPWM(self):
        right_pwm = 0.0
        left_pwm = 0.0
        for motor in self.motors_dict.values():
            if motor.side == "right":
                right_pwm = float(motor.current_pwm)
            elif motor.side == "left":
                left_pwm = float(motor.current_pwm)
        return right_pwm, left_pwm

    def __toMotorObjects(self, yaml_data: dict) -> dict:
        motors = {}
        for motor_name, motor_config in yaml_data.items():
            motors[motor_name] = Motors(motor_config)
        return motors