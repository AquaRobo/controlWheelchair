from control.services.Navigation import Navigation
from control.services.MotorSpeedEvaluator import MotorSpeedEvaluator
from control.services.PIDController import PIDController
from utils.Configurator import Configurator
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from my_robot_interfaces.msg import MotorSpeeds
import numpy as np
import rclpy

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.pid_params = Configurator("control").fetchData(Configurator.PID_PARAMS)
        self._logger = self.get_logger()
        self._logger.info(f"PID Parameters: {self.pid_params}")
        self.pid_yaw = PIDController(self.pid_params["yaw_KP"], self.pid_params["yaw_KI"], self.pid_params["yaw_KD"], setpoint=None)
        self.speed_evaluator = MotorSpeedEvaluator()
        self.navigation = Navigation(self.speed_evaluator)

        self.x_axis = None
        self.z_axis = None
        self.yaw = None
        self.heading_latched = False
        self.wheel_speeds_msg = Float64MultiArray()
        self.motor_speeds_msg = MotorSpeeds()

        self.cmd_vel_subscription = self.create_subscription(Twist, 'cmd_vel', self._cmdVelCallback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu', self._imuCallback, 10)
        self.wheel_speeds_pub = self.create_publisher(Float64MultiArray, '/simple_velocity_controller/commands', 10)
        self.motor_speeds_pub = self.create_publisher(MotorSpeeds, '/motor_speeds', 10)
        self.timer = self.create_timer(0.1, self.navigate)  # Run at 10 Hz

    def _cmdVelCallback(self, msg: Twist) -> None:
        self.x_axis = -msg.linear.x
        self.z_axis = msg.angular.z

    def _imuCallback(self, msg: Imu) -> None:
        # Extract yaw from quaternion
        siny_cosp = 2 * (msg.orientation.w * msg.orientation.z + msg.orientation.x * msg.orientation.y)
        cosy_cosp = 1 - 2 * (msg.orientation.y * msg.orientation.y + msg.orientation.z * msg.orientation.z)
        self.yaw = np.degrees(np.arctan2(siny_cosp, cosy_cosp))

    def __isMovingHorizontal(self) -> bool:
        return (abs(self.x_axis) > 0.1 and abs(self.z_axis) < 0.1)
    
    def __isYawControllerReady(self) -> bool:
        return (self.yaw is not None and self.pid_yaw.setpoint is not None)
    
    def _stabalizeHorizontal(self) -> None:
        yaw_output = self.pid_yaw.stablize(self.yaw)
        # self._logger.info(f"Yaw Setpoint: {self.pid_yaw.setpoint}, Current Yaw: {self.yaw}, PID Output: {yaw_output}")
        self.navigation.navigate(self.x_axis, self.z_axis, yaw_output)
        self.publishOnWheels()

    def navigate(self):
        if self.x_axis is not None and self.z_axis is not None:
            if self.__isMovingHorizontal():
                if not self.heading_latched and self.yaw is not None:
                    self.pid_yaw.updateSetpoint(self.yaw)
                    self.heading_latched = True
                if self.__isYawControllerReady():
                    self._stabalizeHorizontal()
                else:
                    self.navigation.navigate(self.x_axis, self.z_axis, 0.0)
                    self.publishOnWheels()
            else:
                self.heading_latched = False
                self.pid_yaw.setpoint = None
                self.navigation.navigate(self.x_axis, self.z_axis, 0.0)
                self.publishOnWheels()

    def publishOnWheels(self):
        self.wheel_speeds_msg.data = self.navigation.getMotorsSpeed()
        self.wheel_speeds_pub.publish(self.wheel_speeds_msg)
        right_pwm, left_pwm = self.navigation.getMotorsPWM()
        self.motor_speeds_msg.right_motor = right_pwm
        self.motor_speeds_msg.left_motor = left_pwm
        self.motor_speeds_pub.publish(self.motor_speeds_msg)

def main(args=None):
    rclpy.init(args=args)
    navigation_node = NavigationNode()
    try:
        rclpy.spin(navigation_node)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
