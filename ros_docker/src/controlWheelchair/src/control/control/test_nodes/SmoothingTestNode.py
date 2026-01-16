from rclpy.node import Node
from control.services.PWMMapper import PWMMapper
from control.services.MotorSpeedEvaluator import MotorSpeedEvaluator
from control.DTOs.motors import Motors
from utils.Configurator import Configurator
from utils.Dispatcher import Dispatcher
from geometry_msgs.msg import Twist
import rclpy

class SmoothingTestNode(Node):
    def __init__(self):
        super().__init__('smoothing_test_node')
        motor_config = Configurator("control").fetchData(Configurator.MOTORS)
        self.motors = self.__toMotorObjects(motor_config)
        self.smoother = Dispatcher().get_smoothing_strategy()
        self.speed_evaluator = MotorSpeedEvaluator()
        self._logger = self.get_logger()

        self.x_axis = None
        self.z_axis = None
        self.yaw = 0.0
        self.cmd_vel_subscription = self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_callback, 10)
        self.timer = self.create_timer(0.1, self.testSmoothing)

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.x_axis = msg.linear.x
        self.z_axis = msg.angular.z

    def printMotors(self):
        for motor_name, motor in self.motors.items():
            self._logger.info(f"Motor: {motor_name}, Speed: {motor.speed}, Current_PWM: {motor.current_pwm}, Target PWM: {motor.target_pwm}, Dir Bit: {motor.dir_bit}, Side: {motor.side}")

    def __toMotorObjects(self, yaml_data: dict) -> dict:
        motors = {}
        for motor_name, motor_config in yaml_data.items():
            motors[motor_name] = Motors(motor_config)
        return motors
    
    def testSmoothing(self):
        if self.x_axis is not None and self.z_axis is not None:
            self.speed_evaluator.evaluateSpeeds(self.x_axis, self.z_axis, self.yaw, self.motors)
            PWMMapper().mapAxesToPWM(self.motors)
            for name, motor in self.motors.items():
                target_pwm = motor.target_pwm
                smoothed_pwm = self.smoother.smooth(motor.current_pwm, target_pwm)
                motor.current_pwm = smoothed_pwm
                self.printMotors() 


def main(args=None):
    rclpy.init(args=args)
    smoothing_test_node = SmoothingTestNode()
    try:
        rclpy.spin(smoothing_test_node)
    except Exception as e:
        smoothing_test_node._logger.info(f"SmoothingTestNode stopped: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        smoothing_test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()