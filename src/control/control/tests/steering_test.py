from control.strategies.DifferentialDriveSteering import DifferentialDriveSteering
from control.DTOs.motors import Motors
from utils.Configurator import Configurator

class TestDifferentialDriveSteering:
    def __init__(self):
        motor_config = Configurator("control").fetchData(Configurator.MOTORS)
        self.motors = self.__toMotorObjects(motor_config)

    def printMotors(self):
        for motor_name, motor in self.motors.items():
            print(f"Motor: {motor_name}, Speed: {motor.speed}, Current PWM: {motor.current_pwm}, Dir Bit: {motor.dir_bit}, Side: {motor.side}")

    def __toMotorObjects(self, yaml_data: dict) -> dict:
        motors = {}
        for motor_name, motor_config in yaml_data.items():
            motors[motor_name] = Motors(motor_config)
        return motors
    
    def test_steer_forward(self):
        DifferentialDriveSteering.steer(x_axis=1.0, z_axis=0.0, motors_dict=self.motors)
        print("After steering forward:")
        self.printMotors()

    def test_steer_backward(self):
        DifferentialDriveSteering.steer(x_axis=-1.0, z_axis=0.0, motors_dict=self.motors)
        print("After steering backward:")
        self.printMotors()

    def test_steer_left(self):
        DifferentialDriveSteering.steer(x_axis=0.0, z_axis=1.0, motors_dict=self.motors)
        print("After steering left:")
        self.printMotors()

    def test_steer_right(self):
        DifferentialDriveSteering.steer(x_axis=0.0, z_axis=-1.0, motors_dict=self.motors)
        print("After steering right:")
        self.printMotors()


if __name__ == "__main__":
    tester = TestDifferentialDriveSteering()
    tester.test_steer_forward()
    tester.test_steer_backward()
    tester.test_steer_left()
    tester.test_steer_right()