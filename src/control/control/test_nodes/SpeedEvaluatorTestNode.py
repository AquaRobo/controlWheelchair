from control.services.MotorSpeedEvaluator import MotorSpeedEvaluator
from utils.Configurator import Configurator
from control.DTOs.motors import Motors
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import rclpy

class SpeedEvaluatorTestNode(Node):
    def __init__(self):
        super().__init__('speed_evaluator_test_node')
        self._logger = self.get_logger()
        motor_config = Configurator("control").fetchData(Configurator.MOTORS)
        self.speed_evaluator = MotorSpeedEvaluator()
        self.motors = self.__toMotorObjects(motor_config)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.publisher = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.timer_ = self.create_timer(1.0, self.publish_joint_states)

    def cmd_vel_callback(self, msg: Twist):
        # self._logger.info(f"Received cmd_vel: {msg}")
        x_axis = msg.linear.x
        z_axis = -msg.angular.z
        yaw_axis = 0.0  # Assuming no yaw control signal for this test
        self.speed_evaluator.evaluateSpeeds(x_axis, z_axis, yaw_axis, self.motors)
        # self._logger.info("After evaluating speeds:")
        # self.printMotors()

    def printMotors(self):
        for motor_name, motor in self.motors.items():
            self._logger.info(f"Motor: {motor_name}, Speed: {motor.speed}, Current PWM: {motor.current_pwm}, Dir Bit: {motor.dir_bit}, Side: {motor.side}")

    def __toMotorObjects(self, yaml_data: dict) -> dict:
        motors = {}
        for motor_name, motor_config in yaml_data.items():
            motors[motor_name] = Motors(motor_config)
        return motors
    
    def publish_joint_states(self):
        left_motor_speed = self.motors['LEFT_MOTORS'].speed
        right_motor_speed = self.motors['RIGHT_MOTORS'].speed
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [left_motor_speed, right_motor_speed]
        self.publisher.publish(wheel_speed_msg)
        # self._logger.info(f"Published wheel speeds: {wheel_speed_msg}")
    
def main(args=None):
    rclpy.init(args=args)
    try:
        speed_evaluator_test_node = SpeedEvaluatorTestNode()
        rclpy.spin(speed_evaluator_test_node)
    except Exception as e:
        speed_evaluator_test_node.get_logger().error(f"Error occurred: {e}")
    finally:
        speed_evaluator_test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()