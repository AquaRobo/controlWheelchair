from control.strategies.DifferentialDriveSteering import DifferentialDriveSteering
from control.services.MotorSpeedEvaluator import MotorSpeedEvaluator
from control.DTOs.motors import Motors
from utils.Configurator import Configurator
from rclpy.node import Node
from geometry_msgs.msg import Twist
import rclpy

class SteeringTestNode(Node):
    def __init__(self):
        super().__init__('steering_test_node')
        self._logger = self.get_logger()
        motor_config = Configurator("control").fetchData(Configurator.MOTORS)
        self.speed_evaluator = MotorSpeedEvaluator()
        self.motors = self.__toMotorObjects(motor_config)
        self.x_axis = None
        self.z_axis = None
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self._timer = self.create_timer(0.1, self.run)

    def cmd_vel_callback(self, msg: Twist):
        self.x_axis = msg.linear.x
        self.z_axis = msg.angular.z

    def run(self):
        if self.x_axis is not None and self.z_axis is not None:
            self.speed_evaluator.evaluateSpeeds(self.x_axis, self.z_axis, 0.0 ,self.motors)
            DifferentialDriveSteering.steer(self.motors)
            self.printMotors()

    def printMotors(self):
        for motor_name, motor in self.motors.items():
            self._logger.info(f"Motor: {motor_name}, Speed: {motor.speed}, Dir Bit: {motor.dir_bit}, Side: {motor.side}")

    def __toMotorObjects(self, yaml_data: dict) -> dict:
        motors = {}
        for motor_name, motor_config in yaml_data.items():
            motors[motor_name] = Motors(motor_config)
        return motors
    
def main(args=None):
    rclpy.init(args=args)
    try:
        steering_test_node = SteeringTestNode()
        rclpy.spin(steering_test_node)
    except Exception as e:
        steering_test_node.get_logger().error(f"Error occurred: {e}")
    finally:
        steering_test_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()