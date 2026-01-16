from control.interfaces.IMotorDriver import IMotorDriver
from control.interfaces.ISpeedEvaluator import ISpeedEvaluator
from control.services.PWMMapper import PWMMapper
from control.DTOs.motors import Motors
from utils.Dispatcher import Dispatcher
from utils.Configurator import Configurator

class Navigation:
    def __init__(self, motor_driver: IMotorDriver, speed_evaluator: ISpeedEvaluator):
        motors_yaml_data = Configurator("control").fetchData(Configurator.MOTORS)
        self.motor_driver = motor_driver
        self.speed_evaluator = speed_evaluator
        self.steering_strat = Dispatcher().get_steering_strategy()
        self.smoothing_strat = Dispatcher().get_smoothing_strategy()
        # Convert YAML dict to Motor objects
        self.motors_dict = self.__toMotorObjects(motors_yaml_data)

    def navigate(self, x_axis: float, z_axis: float, yaw_axis: float) -> None:
        self.speed_evaluator.evaluateSpeeds(x_axis, z_axis, yaw_axis, self.motors_dict)
        self.steering_strat.steer(self.motors_dict)
        PWMMapper().mapAxesToPWM(self.motors_dict)
        for motor_name, motor in self.motors_dict.items():
            # print(f"Motor: {motor_name}, Speed: {motor.speed}, Current PWM: {motor.current_pwm}, Dir Bit: {motor.dir_bit}")
            target_pwm = motor.target_pwm
            smoothed_pwm = self.smoothing_strat.smooth(motor.current_pwm, target_pwm)
            motor.current_pwm = smoothed_pwm
        self.motor_driver.drive(self.motors_dict)

    def getMotorsSpeed(self):
        speeds = []
        for motor_name, motor in self.motors_dict.items():
            speeds.append(motor.speed)
        return speeds

    def __toMotorObjects(self, yaml_data: dict) -> dict:
        motors = {}
        for motor_name, motor_config in yaml_data.items():
            motors[motor_name] = Motors(motor_config)
        return motors