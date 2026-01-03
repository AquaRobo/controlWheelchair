from control.services.MotorSpeedEvaluator import MotorSpeedEvaluator
from control.DTOs.motors import Motors
from utils.Configurator import Configurator

class TestMotorSpeedEvaluator:

    def __init__(self):
        motor_config = Configurator("control").fetchData(Configurator.MOTORS)
        self.speed_evaluator = MotorSpeedEvaluator()
        self.motors = self.__toMotorObjects(motor_config)

    def printMotors(self):
        for motor_name, motor in self.motors.items():
            print(f"Motor: {motor_name}, Speed: {motor.speed}, Current PWM: {motor.current_pwm}, Dir Bit: {motor.dir_bit}, Side: {motor.side}")

    def __toMotorObjects(self, yaml_data: dict) -> dict:
        motors = {}
        for motor_name, motor_config in yaml_data.items():
            motors[motor_name] = Motors(motor_config)
        return motors

    def test_evaluate_speeds_forward(self):
        self.speed_evaluator.evaluateSpeeds(x_axis=1.0, z_axis=0.0, yaw_axis=0.0, motors_dict=self.motors)
        print("After evaluating speeds forward:")
        self.printMotors()

    def test_evaluate_speeds_backward(self):
        self.speed_evaluator.evaluateSpeeds(x_axis=-1.0, z_axis=0.0, yaw_axis=0.0, motors_dict=self.motors)
        print("After evaluating speeds backward:")
        self.printMotors()

    def test_evaluate_speeds_turn_left(self):
        self.speed_evaluator.evaluateSpeeds(x_axis=0.0, z_axis=1.0, yaw_axis=0.0, motors_dict=self.motors)
        print("After evaluating speeds turn left:")
        self.printMotors()

    def test_evaluate_speeds_turn_right(self):
        self.speed_evaluator.evaluateSpeeds(x_axis=0.0, z_axis=-1.0, yaw_axis=0.0, motors_dict=self.motors)
        print("After evaluating speeds turn right:")
        self.printMotors()


if __name__ == "__main__":
    tester = TestMotorSpeedEvaluator()
    tester.test_evaluate_speeds_forward()
    tester.test_evaluate_speeds_backward()
    tester.test_evaluate_speeds_turn_left()
    tester.test_evaluate_speeds_turn_right()